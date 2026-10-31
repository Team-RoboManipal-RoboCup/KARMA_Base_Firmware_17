/*
 * motor.h
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

// Configuration struct to keep motor driver hardware-agnostic
typedef struct {
    uint32_t      tim_channel;
    GPIO_TypeDef* dir_port;
    uint16_t      dir_pin;
} motor_config_t;

#define MOTOR_MAX_PWM_LIMIT  500  // Limit max PWM to under 50% duty cycle (-500..+500)

void motor_init(TIM_HandleTypeDef *htim_pwm, motor_config_t *configs);
void motor_set(uint8_t motor_id, int16_t speed);  // speed: -1000..1000 (clamped to +/-MOTOR_MAX_PWM_LIMIT)
void motor_stop_all(void);

#endif/* INC_MOTOR_H_ */
