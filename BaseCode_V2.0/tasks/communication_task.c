#include "tasks/communication_task.h"
#include "tasks/task_manager.h"
#include "tasks/jsmn.h"
#include "usbd_cdc_if.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CMD_LEN 128
#define MAX_TX_LEN  256
#define NUM_JSMN_TOKENS 32

static char s_rx_line[MAX_CMD_LEN];
static uint16_t s_rx_idx = 0;

static char s_telemetry_buf[MAX_TX_LEN];
static char s_ack_buf[MAX_CMD_LEN];

static bool jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, (size_t)(tok->end - tok->start)) == 0) {
        return true;
    }
    return false;
}

static int parse_token_int(const char *json, jsmntok_t *tok)
{
    char tmp[16];
    int len = tok->end - tok->start;
    if (len >= (int)sizeof(tmp)) len = sizeof(tmp) - 1;
    memcpy(tmp, json + tok->start, (size_t)len);
    tmp[len] = '\0';
    return atoi(tmp);
}

static void usb_transmit_safe(const char *str, uint16_t len)
{
    if (!str || len == 0) return;

    /* Block with FreeRTOS binary semaphore until previous transmission completes */
    if (xSemaphoreTake(xUsbTxSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (CDC_Transmit_FS((uint8_t *)str, len) != USBD_OK) {
            /* If transmit failed to initiate, immediately release semaphore */
            xSemaphoreGive(xUsbTxSemaphore);
        }
    }
}

static void process_incoming_command(const char *cmd)
{
    /* Emergency stop check */
    if (strstr(cmd, "stop") != NULL) {
        task_manager_notify_jetson_packet_received();
        motor_cmd_t m_cmd = { .speeds = {0, 0, 0, 0}, .timestamp_tick = xTaskGetTickCount() };
        xQueueOverwrite(q_motor_cmd, &m_cmd);

        snprintf(s_ack_buf, sizeof(s_ack_buf), "{\"status\":\"OK\",\"cmd\":\"stop\",\"pwm\":[0,0,0,0]}\r\n");
        usb_transmit_safe(s_ack_buf, (uint16_t)strlen(s_ack_buf));
        return;
    }

    /* Zero-allocation JSMN tokenizer */
    jsmn_parser parser;
    jsmntok_t tokens[NUM_JSMN_TOKENS];
    jsmn_init(&parser);

    int r = jsmn_parse(&parser, cmd, strlen(cmd), tokens, NUM_JSMN_TOKENS);
    bool valid_cmd = false;
    motor_cmd_t m_cmd;
    m_cmd.timestamp_tick = xTaskGetTickCount();

    /* Start with current speeds */
    robot_state_t state_now;
    task_manager_get_state_snapshot(&state_now);
    for (int i = 0; i < NUM_MOTORS; i++) {
        m_cmd.speeds[i] = state_now.motor_pwms[i];
    }

    if (r > 0) {
        for (int i = 1; i < r; i++) {
            if (jsoneq(cmd, &tokens[i], "m1") && (i + 1 < r)) {
                m_cmd.speeds[0] = (int16_t)parse_token_int(cmd, &tokens[++i]);
                valid_cmd = true;
            } else if (jsoneq(cmd, &tokens[i], "m2") && (i + 1 < r)) {
                m_cmd.speeds[1] = (int16_t)parse_token_int(cmd, &tokens[++i]);
                valid_cmd = true;
            } else if (jsoneq(cmd, &tokens[i], "m3") && (i + 1 < r)) {
                m_cmd.speeds[2] = (int16_t)parse_token_int(cmd, &tokens[++i]);
                valid_cmd = true;
            } else if (jsoneq(cmd, &tokens[i], "m4") && (i + 1 < r)) {
                m_cmd.speeds[3] = (int16_t)parse_token_int(cmd, &tokens[++i]);
                valid_cmd = true;
            } else if ((jsoneq(cmd, &tokens[i], "speed") || jsoneq(cmd, &tokens[i], "pwm")) && (i + 1 < r)) {
                if (tokens[i + 1].type == JSMN_ARRAY) {
                    /* Array format: {"pwm": [500, 500, -500, -500]} */
                    i++; /* move to array token */
                    int arr_len = tokens[i].size;
                    for (int k = 0; k < arr_len && k < NUM_MOTORS; k++) {
                        m_cmd.speeds[k] = (int16_t)parse_token_int(cmd, &tokens[++i]);
                    }
                    valid_cmd = true;
                } else {
                    /* Single broadcast speed: {"speed": 500} */
                    int16_t broadcast = (int16_t)parse_token_int(cmd, &tokens[++i]);
                    for (int k = 0; k < NUM_MOTORS; k++) {
                        m_cmd.speeds[k] = broadcast;
                    }
                    valid_cmd = true;
                }
            }
        }
    }

    /* Fallback: raw numeric string (e.g. "50\n" from terminal) */
    if (!valid_cmd) {
        const char *c = cmd;
        while (*c == ' ' || *c == '\t') c++;
        if (*c == '-' || (*c >= '0' && *c <= '9')) {
            int raw_val = atoi(c);
            if (raw_val >= -100 && raw_val <= 100) {
                raw_val *= 10;
            }
            for (int k = 0; k < NUM_MOTORS; k++) {
                m_cmd.speeds[k] = (int16_t)raw_val;
            }
            valid_cmd = true;
        }
    }

    if (valid_cmd) {
        /* Update deadman watchdog timestamp */
        task_manager_notify_jetson_packet_received();

        /* Push command to MotorTask */
        xQueueOverwrite(q_motor_cmd, &m_cmd);

        /* Send ACK */
        snprintf(s_ack_buf, sizeof(s_ack_buf),
                 "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n",
                 m_cmd.speeds[0], m_cmd.speeds[1], m_cmd.speeds[2], m_cmd.speeds[3]);
        usb_transmit_safe(s_ack_buf, (uint16_t)strlen(s_ack_buf));
    } else {
        snprintf(s_ack_buf, sizeof(s_ack_buf), "{\"status\":\"ERR\",\"msg\":\"unknown cmd\"}\r\n");
        usb_transmit_safe(s_ack_buf, (uint16_t)strlen(s_ack_buf));
    }
}

void communication_task_entry(void *argument)
{
    (void)argument;

    TickType_t xLastTelemetryTime = xTaskGetTickCount();
    const TickType_t xTelemetryPeriod = pdMS_TO_TICKS(20); /* 50 Hz / 20 ms */

    robot_state_t state_snapshot;
    uint8_t rx_byte;

    for (;;) {
        /* 1. Process inbound bytes from USB RX queue */
        while (xQueueReceive(q_usb_rx, &rx_byte, 0) == pdPASS) {
            char c = (char)rx_byte;
            if (c == '\r' || c == '\n') {
                if (s_rx_idx > 0) {
                    s_rx_line[s_rx_idx] = '\0';
                    process_incoming_command(s_rx_line);
                    s_rx_idx = 0;
                }
            } else if (s_rx_idx < (sizeof(s_rx_line) - 1)) {
                s_rx_line[s_rx_idx++] = c;
            }
        }

        /* 2. Periodic 50 Hz JSON Telemetry output */
        TickType_t now = xTaskGetTickCount();
        if ((now - xLastTelemetryTime) >= xTelemetryPeriod) {
            xLastTelemetryTime = now;

            /* Grab mutex-protected snapshot */
            task_manager_get_state_snapshot(&state_snapshot);

            /* Format JSON */
            int len = snprintf(
                s_telemetry_buf, sizeof(s_telemetry_buf),
                "{\"yaw\":%.2f,\"x\":%.2f,\"y\":%.2f,\"v\":%.2f,\"w\":%.2f,"
                "\"bno_ok\":%d,\"pwm\":[%d,%d,%d,%d]}\r\n",
                state_snapshot.pose.heading_deg,
                state_snapshot.pose.x_mm,
                state_snapshot.pose.y_mm,
                state_snapshot.pose.linear_vel_m_s,
                state_snapshot.pose.angular_vel_rad_s,
                state_snapshot.bno_ready ? 1 : 0,
                state_snapshot.motor_pwms[0],
                state_snapshot.motor_pwms[1],
                state_snapshot.motor_pwms[2],
                state_snapshot.motor_pwms[3]
            );

            if (len > 0 && len < (int)sizeof(s_telemetry_buf)) {
                usb_transmit_safe(s_telemetry_buf, (uint16_t)len);
            }
        }

        /* Yield briefly to lower priority tasks if no incoming data */
        vTaskDelay(pdMS_TO_TICKS(2));

        /* Stack watermarking */
        UBaseType_t high_water_words = uxTaskGetStackHighWaterMark(NULL);
        if (high_water_words < 50) {
            logger_log(LOG_LEVEL_ERROR, "CommunicationTask low stack margin: %lu words!", (unsigned long)high_water_words);
        }
    }
}
