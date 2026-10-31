#include "tasks/estimation_task.h"
#include "tasks/task_manager.h"
#include "algorithm.h"
#include "drivers/motor_driver.h"
#include "logger.h"

void estimation_task_entry(void *argument)
{
    (void)argument;

    robot_pose_t current_pose;
    algorithm_init(&current_pose);

    sensor_data_t sensor_packet;
    robot_state_t state_packet;

    for (;;) {
        /* Wait for new sensor data packet from AcquisitionTask */
        if (xQueueReceive(q_sensor_data, &sensor_packet, portMAX_DELAY) == pdPASS) {
            /* 1. Run dead reckoning math (R = 35 mm, W = 150 mm) */
            /* Channel 0: Left dead wheel, Channel 1: Right dead wheel, Channel 2: Aux/perpendicular */
            int16_t dl = sensor_packet.encoders.deltas[0];
            int16_t dr = sensor_packet.encoders.deltas[1];
            int16_t da = sensor_packet.encoders.deltas[2];

            algorithm_update_odometry(
                &current_pose,
                dl, dr, da,
                0.020f, /* 50 Hz dt = 20 ms */
                sensor_packet.imu.yaw,
                sensor_packet.imu.valid
            );

            /* 2. Assemble robot state */
            state_packet.pose           = current_pose;
            state_packet.bno_ready      = sensor_packet.imu.valid;
            state_packet.timestamp_tick = sensor_packet.timestamp_tick;
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                state_packet.motor_pwms[i] = motor_driver_get_speed(i);
            }

            /* 3. Update thread-safe snapshot for CommunicationTask / Telemetry */
            task_manager_set_state_snapshot(&state_packet);

            /* 4. Push to robot state queue */
            xQueueSend(q_robot_state, &state_packet, 0);

            /* 5. Instrument stack safety */
            UBaseType_t high_water_words = uxTaskGetStackHighWaterMark(NULL);
            if (high_water_words < 50) {
                logger_log(LOG_LEVEL_ERROR, "EstimationTask low stack margin: %lu words!", (unsigned long)high_water_words);
            }
        }
    }
}
