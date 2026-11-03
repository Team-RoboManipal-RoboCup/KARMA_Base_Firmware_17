/*
 * motor.h
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

#define MOTOR_COUNT             4U
#define MOTOR_MAX_PWM_LIMIT     500

/* Motor hardware configuration */
typedef struct
{
    uint32_t tim_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

} motor_config_t;


/* Motor functions */
void motor_init(
    TIM_HandleTypeDef *htim_pwm,
    const motor_config_t *configs
);

void motor_set(uint8_t motor_id, int16_t speed);

void motor_stop_all(void);

#endif
