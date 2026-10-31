/*
 * robot.c
 *
 * Implementation of Autonomous 4-Wheel Robot Control
 * Integrates:
 *  - 3-Encoder Dead Wheel Odometry (TIM4, TIM3, TIM2)
 *  - BNO085 IMU (Game Rotation Vector for Yaw, Linear Accel / Accel for x, y,
 * z)
 *  - 4x MD30C Motor Drivers (TIM5 CH1..CH4 PWM, GPIOB DIR)
 *  - USB CDC JSON Bidirectional Telemetry and Command Parsing
 */

#include "robot.h"
#include "bno085_driver.h"
#include "bno_port.h"
#include "encoder.h"
#include "main.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Hardware Timer Handles declared in main.c */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* BNO085 instances */
static BNO085 s_bno;
static BNO_Port s_bno_port;
static bool s_bno_ready = false;

/* State variables */
static uint32_t s_last_telemetry_tick = 0;
static uint32_t s_last_cmd_tick = 0;
static bool s_motors_active = false;
static int16_t s_motor_pwm[4] = {0, 0, 0, 0};

/* USB Buffers */
static char s_tx_buf[256];
static char s_ack_buf[128];
static volatile bool s_pending_ack = false;
static char s_rx_line[128];
static uint8_t s_rx_idx = 0;

/* Helper to parse an integer following a key in a JSON-like string */
static bool parse_json_int(const char *str, const char *key, int *out_val) {
  const char *p = strstr(str, key);
  if (!p)
    return false;

  p += strlen(key);
  while (*p && (*p == ' ' || *p == ':' || *p == '"'))
    p++;

  if (*p == '-' || (*p >= '0' && *p <= '9')) {
    *out_val = atoi(p);
    return true;
  }
  return false;
}

void robot_init(void) {
  /* 1. Initialize 3 Dead Wheel Encoders (TIM4, TIM3, TIM2) */
  encoder_init(&htim4, &htim3, &htim2);

  /* 2. Initialize 4x MD30C Motor Drivers (PWM on TIM5 CH1..CH4, DIR on GPIOB)
   */
  motor_config_t my_motors[4] = {
      {TIM_CHANNEL_3, DIR1_GPIO_Port, DIR1_Pin}, /* M1: PA0 (TIM5_CH3), DIR: PB0 */
      {TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin}, /* M2: PA1 (TIM5_CH2), DIR: PB1 */
      {TIM_CHANNEL_1, DIR3_GPIO_Port, DIR3_Pin}, /* M3: PA2 (TIM5_CH1), DIR: PB2 */
      {TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin}  /* M4: PA3 (TIM5_CH4), DIR: PB12 */
  };
  motor_init(&htim5, my_motors);

  /* 3. Initialize BNO085 IMU */
  BNO_Port_Init(&s_bno_port);
  if (BNO_Port_Detect()) {
    if (BNO085_Init(&s_bno, &s_bno_port) == BNO_PORT_OK) {
      /* Drain initial startup and advertisement SHTP packets from BNO boot */
      for (int i = 0; i < 15; i++) {
        BNO085_Update(&s_bno);
        HAL_Delay(10);
      }

      /* Enable Game Rotation Vector (50Hz / 20000us) for drift-free yaw */
      BNO085_EnableGameRotation(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable standard Rotation Vector as fallback */
      BNO085_EnableRotation(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable Linear Acceleration (50Hz / 20000us) for gravity-free x, y, z */
      BNO085_EnableLinearAccel(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable standard Acceleration as fallback */
      BNO085_EnableAccel(&s_bno, 20000);
      HAL_Delay(20);

      s_bno_ready = true;
    }
  }

  s_last_telemetry_tick = HAL_GetTick();
  s_last_cmd_tick = HAL_GetTick();
}

void robot_set_motor(uint8_t motor_id, int16_t speed) {
  if (motor_id > 3)
    return;
  if (speed > 1000)
    speed = 1000;
  if (speed < -1000)
    speed = -1000;
  s_motor_pwm[motor_id] = speed;
  motor_set(motor_id, speed);
  if (speed != 0)
    s_motors_active = true;
}

void robot_set_all_motors(int16_t speed) {
  for (int i = 0; i < 4; i++) {
    robot_set_motor(i, speed);
  }
}

void robot_stop(void) {
  for (int i = 0; i < 4; i++) {
    s_motor_pwm[i] = 0;
  }
  motor_stop_all();
  s_motors_active = false;
}

int16_t robot_get_motor_pwm(uint8_t motor_id) {
  if (motor_id < 4)
    return s_motor_pwm[motor_id];
  return 0;
}

/*
 * Parses incoming commands over USB CDC.
 * Supports:
 *   1. Individual JSON keys: {"m1": 500, "m2": 500, "m3": -500, "m4": -500}
 *   2. Array format:         {"m": [500, 500, -500, -500]} or {"pwm": [500,
 * 500, -500, -500]}
 *   3. Broadcast speed:      {"speed": 300} or {"pwm": 300}
 *   4. Emergency stop:       {"cmd": "stop"} or "stop"
 *   5. Raw number input:     "50" (sets all motors to 50% / 500 speed)
 */
static void parse_command(const char *cmd) {
  s_last_cmd_tick = HAL_GetTick();

  /* Emergency stop check */
  if (strstr(cmd, "stop") != NULL) {
    robot_stop();
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"cmd\":\"stop\",\"pwm\":[0,0,0,0]}\r\n");
    s_pending_ack = true;
    return;
  }

  /* 1. Check for individual motor keys "m1", "m2", "m3", "m4" */
  int val;
  bool found_individual = false;
  if (parse_json_int(cmd, "\"m1\"", &val)) {
    robot_set_motor(0, (int16_t)val);
    found_individual = true;
  }
  if (parse_json_int(cmd, "\"m2\"", &val)) {
    robot_set_motor(1, (int16_t)val);
    found_individual = true;
  }
  if (parse_json_int(cmd, "\"m3\"", &val)) {
    robot_set_motor(2, (int16_t)val);
    found_individual = true;
  }
  if (parse_json_int(cmd, "\"m4\"", &val)) {
    robot_set_motor(3, (int16_t)val);
    found_individual = true;
  }
  if (found_individual) {
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
             s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
    s_pending_ack = true;
    return;
  }

  /* 2. Check for array format: {"m": [...]} or {"pwm": [...]} */
  const char *p = strstr(cmd, "\"m\":");
  if (!p)
    p = strstr(cmd, "\"pwm\":");
  if (p) {
    const char *bracket = strchr(p, '[');
    if (bracket) {
      bracket++;
      for (int i = 0; i < 4; i++) {
        while (*bracket == ' ' || *bracket == ',')
          bracket++;
        if (!*bracket || *bracket == ']')
          break;
        int spd = atoi(bracket);
        robot_set_motor(i, (int16_t)spd);
        while (*bracket && *bracket != ',' && *bracket != ']')
          bracket++;
        if (*bracket == ',')
          bracket++;
      }
      snprintf(s_ack_buf, sizeof(s_ack_buf),
               "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
               s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
      s_pending_ack = true;
      return;
    } else {
      /* Single broadcast value: {"pwm": 500} */
      const char *col = strchr(p, ':');
      if (col && parse_json_int(col - 5, "\"pwm\"", &val)) {
        robot_set_all_motors((int16_t)val);
        snprintf(s_ack_buf, sizeof(s_ack_buf),
                 "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n",
                 s_motor_pwm[0], s_motor_pwm[1], s_motor_pwm[2],
                 s_motor_pwm[3]);
        s_pending_ack = true;
        return;
      }
    }
  }

  /* 3. Check for broadcast speed: {"speed": 500} */
  if (parse_json_int(cmd, "\"speed\"", &val)) {
    robot_set_all_motors((int16_t)val);
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
             s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
    s_pending_ack = true;
    return;
  }

  /* 4. Fallback: Raw numeric input (e.g. "50" or "-80" from serial terminal) */
  const char *c = cmd;
  while (*c == ' ' || *c == '\t')
    c++;
  if (*c == '-' || (*c >= '0' && *c <= '9')) {
    int raw_val = atoi(c);
    /* If sent as percentage -100..100, scale to -1000..1000 */
    if (raw_val >= -100 && raw_val <= 100) {
      raw_val *= 10;
    }
    robot_set_all_motors((int16_t)raw_val);
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
             s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
    s_pending_ack = true;
    return;
  }

  /* Unrecognized command */
  snprintf(s_ack_buf, sizeof(s_ack_buf),
           "{\"status\":\"ERR\",\"msg\":\"unknown cmd\"}\r\n");
  s_pending_ack = true;
}

void robot_process_rx(uint8_t *buf, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    char c = (char)buf[i];

    if (c == '\r' || c == '\n') {
      if (s_rx_idx > 0) {
        s_rx_line[s_rx_idx] = '\0';
        parse_command(s_rx_line);
        s_rx_idx = 0;
      }
    } else if (s_rx_idx < (sizeof(s_rx_line) - 1)) {
      s_rx_line[s_rx_idx++] = c;
    }
  }
}

static uint32_t s_last_feature_retry = 0;

void robot_loop(void) {
//1. Process pending BNO085 SHTP packets//
  if (s_bno_ready) {
    for (int i = 0; i < 8; i++) {
      if (BNO085_Update(&s_bno) != BNO_PORT_OK)
        break;
    }

     //If no sensor reports have arrived yet, re-send SetFeature requests every
     //* 250ms

    if (s_bno.diag.rx_reports == 0 &&
        (HAL_GetTick() - s_last_feature_retry >= 250)) {
      s_last_feature_retry = HAL_GetTick();
      BNO085_EnableGameRotation(&s_bno, 20000);
      BNO085_EnableRotation(&s_bno, 20000);
      BNO085_EnableLinearAccel(&s_bno, 20000);
      BNO085_EnableAccel(&s_bno, 20000);
    }
  }

  /* Service immediate command acknowledgment */
  if (s_pending_ack) {
    if (CDC_Transmit_FS((uint8_t *)s_ack_buf, (uint16_t)strlen(s_ack_buf)) ==
        USBD_OK) {
      s_pending_ack = false;
    }
  }

  /* 2. Safety Watchdog: Auto-stop motors if communication drops */
  // #if (MOTOR_WATCHDOG_TIMEOUT > 0)
  //     if (s_motors_active && (HAL_GetTick() - s_last_cmd_tick >=
  //     MOTOR_WATCHDOG_TIMEOUT)) {
  //         robot_stop();
  //     }
  // #endif

  /* 3. Periodic JSON Telemetry Output (50 Hz / 20 ms) */
  if (!s_pending_ack &&
      (HAL_GetTick() - s_last_telemetry_tick >= TELEMETRY_INTERVAL_MS)) {
    s_last_telemetry_tick = HAL_GetTick();

    /* Update 3 dead wheel encoder counts */
    encoder_update();
    int32_t e1 = encoder_get_count(0);
    int32_t e2 = encoder_get_count(1);
    int32_t e3 = encoder_get_count(2);


    //Get BNO085 Heading (Yaw in degrees)//
    BNO_Quaternion q;
    float yaw = 0.0f;
    if (BNO085_GetGameRotation(&s_bno, &q) || BNO085_GetRotation(&s_bno, &q)) {
      yaw = BNO085_GetEulerYaw(&q);
    }

    //Get BNO085 Acceleration (x, y, z in m/s^2)
    BNO_Vector3 accel = {0.0f, 0.0f, 0.0f};
    if (!BNO085_GetLinearAccel(&s_bno, &accel)) {
      BNO085_GetAccel(&s_bno, &accel);
    }


    /* Clean, streamlined JSON telemetry with motor PWM */
    int len = snprintf(
        s_tx_buf, sizeof(s_tx_buf),
        "{\"yaw\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"e1\":%ld,\"e2\":%ld,"
        "\"e3\":%ld,\"bno_ok\":%d,\"pwm\":[%d,%d,%d,%d]}\r\n",
        yaw, accel.x, accel.y, accel.z, (long)e1, (long)e2, (long)e3,
        s_bno_ready ? 1 : 0, s_motor_pwm[0], s_motor_pwm[1], s_motor_pwm[2],
        s_motor_pwm[3]);

    if (len > 0) {
      CDC_Transmit_FS((uint8_t *)s_tx_buf, (uint16_t)len);
    }
  }
}
