#include "logger.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static char              s_ring_buffer[LOG_RING_BUFFER_SIZE];
static uint16_t          s_head = 0;
static uint16_t          s_tail = 0;
static uint16_t          s_count = 0;

static StaticSemaphore_t s_mutex_buffer;
static SemaphoreHandle_t s_log_mutex = NULL;

static StaticTimer_t     s_timer_buffer;
static TimerHandle_t     s_log_timer = NULL;

void logger_init(void)
{
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;

    /* Statically create the mutex */
    s_log_mutex = xSemaphoreCreateMutexStatic(&s_mutex_buffer);

    /* Statically create the 500 ms periodic Software Timer */
    s_log_timer = xTimerCreateStatic(
        "LogTimer",
        pdMS_TO_TICKS(500),
        pdTRUE, /* Auto-reload */
        (void *)0,
        logger_timer_callback,
        &s_timer_buffer
    );

    if (s_log_timer != NULL) {
        xTimerStart(s_log_timer, 0);
    }
}

void logger_log(log_level_t level, const char *fmt, ...)
{
    if (s_log_mutex == NULL) return;

    char temp_buf[128];
    const char *prefix = (level == LOG_LEVEL_ERROR) ? "[ERR] " :
                         (level == LOG_LEVEL_WARN)  ? "[WRN] " : "[INF] ";

    size_t prefix_len = strlen(prefix);
    memcpy(temp_buf, prefix, prefix_len);

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(temp_buf + prefix_len, sizeof(temp_buf) - prefix_len - 2, fmt, args);
    va_end(args);

    if (len < 0) return;
    size_t total_len = prefix_len + (size_t)len;
    temp_buf[total_len++] = '\n';
    temp_buf[total_len]   = '\0';

    /* Mutex-protected write into the static ring buffer */
    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (size_t i = 0; i < total_len; i++) {
            s_ring_buffer[s_head] = temp_buf[i];
            s_head = (s_head + 1) % LOG_RING_BUFFER_SIZE;
            if (s_count < LOG_RING_BUFFER_SIZE) {
                s_count++;
            } else {
                /* Buffer full: advance tail to drop oldest character */
                s_tail = (s_tail + 1) % LOG_RING_BUFFER_SIZE;
            }
        }
        xSemaphoreGive(s_log_mutex);
    }
}

void logger_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    /* 500 ms Software Timer heartbeat / ring buffer monitoring */
    if (s_log_mutex != NULL && xSemaphoreTake(s_log_mutex, 0) == pdTRUE) {
        /* If logs are queued in the ring buffer, they can be processed or audited here */
        xSemaphoreGive(s_log_mutex);
    }
}
