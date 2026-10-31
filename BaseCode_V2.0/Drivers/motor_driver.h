#ifndef DRIVERS_MOTOR_DRIVER_H
#define DRIVERS_MOTOR_DRIVER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MOTORS 4

typedef struct {
    uint32_t      tim_channel;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
} motor_channel_config_t;

/**
 * @brief Initialize 4-channel motor driver with timer handle and pin mappings.
 */
void motor_driver_init(TIM_HandleTypeDef *htim, const motor_channel_config_t *configs);

/**
 * @brief Set speed for a specific motor channel (-1000 to +1000).
 */
void motor_driver_set(uint8_t motor_id, int16_t speed);

/**
 * @brief Set all 4 motor channels simultaneously.
 */
void motor_driver_set_all(const int16_t speeds[NUM_MOTORS]);

/**
 * @brief Immediately stop all 4 motors (zero PWM output).
 */
void motor_driver_stop_all(void);

/**
 * @brief Read back current commanded PWM for telemetry.
 */
int16_t motor_driver_get_speed(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_MOTOR_DRIVER_H */
