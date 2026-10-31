#ifndef TASKS_ACQUISITION_TASK_H
#define TASKS_ACQUISITION_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACQUISITION_TASK_STACK_SIZE 384

/**
 * @brief 50 Hz / 20 ms Sensor Acquisition task (Encoders, BNO085, Optical Flow, ToF).
 */
void acquisition_task_entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_ACQUISITION_TASK_H */
