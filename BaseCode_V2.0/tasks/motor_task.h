#ifndef TASKS_MOTOR_TASK_H
#define TASKS_MOTOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_TASK_STACK_SIZE 256

/**
 * @brief High-priority 100 Hz / 10 ms motor control and 150 ms deadman safety task.
 */
void motor_task_entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_MOTOR_TASK_H */
