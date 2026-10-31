/*
 * motor.c
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#include "motor.h"
#include "main.h"   /* for DIR1_Pin..DIR4_Pin / DIRx_GPIO_Port macros */

typedef struct {
    uint32_t     tim_channel;
    GPIO_TypeDef *dir_port;
    uint16_t     dir_pin;
} motor_map_t;

static TIM_HandleTypeDef *s_htim;

/* Index 0..3 = TIM4 CH1..CH4 = DIR1..DIR4, in that order. This is a
 * convention, not a physical fact -- verify against your wiring diagram
 * and reorder this table if motor_id 0 doesn't correspond to the physical
 * position you expect. */
static const motor_map_t s_motors[4] = {
    { TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin },
    { TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin },
    { TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin },
    { TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin },  /* requires the PB14 relabel above */
};

void motor_init(TIM_HandleTypeDef *htim_pwm)
{
    s_htim = htim_pwm;
    for (int i = 0; i < 4; i++) {
        /* Same gotcha as the encoder side: HAL_TIM_PWM_Init() configures
         * the timer but does not enable the channel output. */
        HAL_TIM_PWM_Start(s_htim, s_motors[i].tim_channel);
    }
    motor_stop_all();
}

void motor_set(uint8_t motor_id, int16_t speed)
{
    if (motor_id > 3) return;
    if (speed >  1000) speed =  1000;
    if (speed < -1000) speed = -1000;

    /* DIR polarity (which level = "forward") is unverified -- confirm
     * empirically per motor and invert here if backwards. Don't assume
     * all 4 channels share the same wiring sense; MD30C boards get
     * mounted in mirrored orientations on opposite sides of a chassis. */
    HAL_GPIO_WritePin(s_motors[motor_id].dir_port, s_motors[motor_id].dir_pin,
                       speed >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint16_t mag = (speed < 0) ? (uint16_t)(-speed) : (uint16_t)speed;
    /* Scaled off the timer's actual Period, so this function's contract
     * (-1000..1000 = full reverse..full forward) doesn't break if you
     * change TIM4's Period in the .ioc for a higher PWM frequency. */
    uint32_t duty = ((uint32_t)mag * s_htim->Init.Period) / 1000;
    __HAL_TIM_SET_COMPARE(s_htim, s_motors[motor_id].tim_channel, duty);
}

void motor_stop_all(void)
{
    for (int i = 0; i < 4; i++) motor_set(i, 0);
}
