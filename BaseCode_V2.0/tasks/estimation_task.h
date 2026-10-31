#ifndef TASKS_ESTIMATION_TASK_H
#define TASKS_ESTIMATION_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESTIMATION_TASK_STACK_SIZE 256

/**
 * @brief 50 Hz Estimation task (Dead reckoning integration, state estimation).
 */
void estimation_task_entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_ESTIMATION_TASK_H */
