#include "tasks/acquisition_task.h"
#include "tasks/task_manager.h"
#include "drivers/hw_encoders.h"
#include "drivers/bno085.h"
#if (ENABLE_OPTICAL_FLOW_TOF == 1)
#include "drivers/optical_flow_tof.h"
#endif
#include "logger.h"

void acquisition_task_entry(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); /* 50 Hz / 20 ms */

#if (ENABLE_OPTICAL_FLOW_TOF == 1)
    optical_flow_tof_init();
#endif
    bno085_hw_init();

    sensor_data_t sensor_packet;

    for (;;) {
        /* Enforce absolute time referencing */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* 1. Update Hardware Encoders */
        hw_encoders_update();
        hw_encoders_get_all(&sensor_packet.encoders);

        /* 2. Step BNO085 State Machine and read orientation/accel */
        bno085_hw_step();
        bno085_hw_get_reading(&sensor_packet.imu);

#if (ENABLE_OPTICAL_FLOW_TOF == 1)
        /* 3. Read Optical Flow and Time-of-Flight (disabled by default) */
        optical_flow_tof_read(&sensor_packet.flow_tof);
#endif

        sensor_packet.timestamp_tick = xTaskGetTickCount();

        /* 4. Push packet to EstimationTask queue (drop oldest if full) */
        if (xQueueSend(q_sensor_data, &sensor_packet, 0) != pdPASS) {
            /* Queue full: pop one stale frame and push fresh frame */
            sensor_data_t dummy;
            xQueueReceive(q_sensor_data, &dummy, 0);
            xQueueSend(q_sensor_data, &sensor_packet, 0);
        }

        /* 5. Stack safety instrumenting */
        UBaseType_t high_water_words = uxTaskGetStackHighWaterMark(NULL);
        if (high_water_words < 50) {
            logger_log(LOG_LEVEL_ERROR, "AcquisitionTask low stack margin: %lu words!", (unsigned long)high_water_words);
        }
    }
}
