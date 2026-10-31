/*
 * motor.h
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void motor_init(TIM_HandleTypeDef *htim_pwm);
void motor_set(uint8_t motor_id, int16_t speed);  // speed: -1000..1000, sign = direction
void motor_stop_all(void);

#endif/* INC_MOTOR_H_ */
