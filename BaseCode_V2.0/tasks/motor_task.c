#include "tasks/motor_task.h"
#include "tasks/task_manager.h"
#include "drivers/motor_driver.h"
#include "logger.h"

void motor_task_entry(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); /* 100 Hz / 10 ms */

    int16_t current_targets[NUM_MOTORS] = {0, 0, 0, 0};
    bool deadman_tripped = false;

    for (;;) {
        /* Enforce absolute time referencing to eliminate loop timing jitter */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        TickType_t current_tick = xTaskGetTickCount();
        uint32_t last_packet_tick = task_manager_get_last_jetson_packet_tick();

        /* 1. Deadman Safety Check: Kill motor power if no command arrived for > 150 ms */
        if ((current_tick - last_packet_tick) > pdMS_TO_TICKS(DEADMAN_TIMEOUT_MS)) {
            if (!deadman_tripped) {
                deadman_tripped = true;
                for (int i = 0; i < NUM_MOTORS; i++) {
                    current_targets[i] = 0;
                }
                motor_driver_stop_all();
                logger_log(LOG_LEVEL_WARN, "Deadman timeout > 150ms! Motors halted.");
            }
        } else {
            deadman_tripped = false;
        }

        /* 2. Check for incoming motor speed commands from q_motor_cmd */
        motor_cmd_t new_cmd;
        if (xQueueReceive(q_motor_cmd, &new_cmd, 0) == pdPASS) {
            if (!deadman_tripped) {
                for (int i = 0; i < NUM_MOTORS; i++) {
                    current_targets[i] = new_cmd.speeds[i];
                }
            }
        }

        /* 3. Output speeds to motor drivers */
        if (!deadman_tripped) {
            motor_driver_set_all(current_targets);
        } else {
            motor_driver_stop_all();
        }

        /* 4. Instrument task stack safety */
        UBaseType_t high_water_words = uxTaskGetStackHighWaterMark(NULL);
        if (high_water_words < 50) {
            logger_log(LOG_LEVEL_ERROR, "MotorTask low stack margin: %lu words!", (unsigned long)high_water_words);
        }
    }
}
