#ifndef DRIVERS_BNO085_H
#define DRIVERS_BNO085_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw;
    float pitch;
    float roll;
    float accel_x;
    float accel_y;
    float accel_z;
    bool  valid;
} bno085_reading_t;

/**
 * @brief Initialize BNO085 driver subsystem and state machine.
 */
void bno085_hw_init(void);

/**
 * @brief Step BNO085 processing loop (runs inside AcquisitionTask).
 */
void bno085_hw_step(void);

/**
 * @brief Retrieve coherent latest BNO085 orientation and acceleration data.
 */
void bno085_hw_get_reading(bno085_reading_t *out_reading);

/**
 * @brief Check if BNO085 has completed initialization and is actively reporting.
 */
bool bno085_hw_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BNO085_H */
