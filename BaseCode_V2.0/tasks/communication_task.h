#ifndef TASKS_COMMUNICATION_TASK_H
#define TASKS_COMMUNICATION_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMMUNICATION_TASK_STACK_SIZE 512

/**
 * @brief Communication Task managing zero-alloc JSON command parsing,
 *        50 Hz telemetry output, and USB-CDC binary semaphore flow control.
 */
void communication_task_entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_COMMUNICATION_TASK_H */
