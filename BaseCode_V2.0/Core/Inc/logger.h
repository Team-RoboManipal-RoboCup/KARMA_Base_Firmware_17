#ifndef LOGGER_H
#define LOGGER_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_RING_BUFFER_SIZE 512

typedef enum {
    LOG_LEVEL_INFO = 0,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

/**
 * @brief Initialize mutex-protected ring buffer and 500 ms static software timer.
 */
void logger_init(void);

/**
 * @brief Thread-safe logging with mutex lock and zero dynamic allocation.
 */
void logger_log(log_level_t level, const char *fmt, ...);

/**
 * @brief Software timer callback (500 ms periodic).
 */
void logger_timer_callback(TimerHandle_t xTimer);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
