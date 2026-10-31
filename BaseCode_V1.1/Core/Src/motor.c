/*
 * motor.c
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#include "motor.h"

static TIM_HandleTypeDef *s_htim;
static motor_config_t s_motors[4];

void motor_init(TIM_HandleTypeDef *htim_pwm, motor_config_t *configs)
{
    s_htim = htim_pwm;
    for (int i = 0; i < 4; i++) {
        s_motors[i] = configs[i];
        HAL_TIM_PWM_Start(s_htim, s_motors[i].tim_channel);
    }
    motor_stop_all();
}

void motor_set(uint8_t motor_id, int16_t speed)
{
    if (motor_id > 3) return;

    // Safety limit: clamp commanded speed to under 50% max PWM
    if (speed >  MOTOR_MAX_PWM_LIMIT) speed =  MOTOR_MAX_PWM_LIMIT;
    if (speed < -MOTOR_MAX_PWM_LIMIT) speed = -MOTOR_MAX_PWM_LIMIT;

    // Set DIR pin state only on active direction commands
    if (speed > 0) {
        HAL_GPIO_WritePin(s_motors[motor_id].dir_port, s_motors[motor_id].dir_pin, GPIO_PIN_SET);
        if (motor_id == 2) {
            /* Fallback mirroring: PB10 (pin next to PB2) and PB15 (V0.0 DIR3 pin) */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_SET);
        }
    } else if (speed < 0) {
        HAL_GPIO_WritePin(s_motors[motor_id].dir_port, s_motors[motor_id].dir_pin, GPIO_PIN_RESET);
        if (motor_id == 2) {
            /* Fallback mirroring: PB10 (pin next to PB2) and PB15 (V0.0 DIR3 pin) */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_15, GPIO_PIN_RESET);
        }
    }

    uint32_t mag = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(s_htim);

    // Clean duty cycle calculation using active ARR
    uint32_t duty = (uint32_t)(((uint64_t)mag * (uint64_t)period) / 1000ULL);

    // Hard ceiling safety check: ensure duty cycle never exceeds 50% of ARR
    if (duty > period / 2) {
        duty = period / 2;
    }

    __HAL_TIM_SET_COMPARE(s_htim, s_motors[motor_id].tim_channel, duty);
}

void motor_stop_all(void)
{
    for (int i = 0; i < 3; i++) motor_set(i, 0); // safe bounds loop
    motor_set(3, 0);
}
