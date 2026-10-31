/*
 * encoder.c
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#include "encoder.h"

static TIM_HandleTypeDef *s_htim[3];
static int32_t  s_total[3];
static uint16_t s_last_raw[3];
static int16_t  s_delta[3];

void encoder_init(TIM_HandleTypeDef *h0, TIM_HandleTypeDef *h1, TIM_HandleTypeDef *h2)
{
    s_htim[0] = h0; s_htim[1] = h1; s_htim[2] = h2;
    for (int i = 0; i < 3; i++) {
        HAL_TIM_Encoder_Start(s_htim[i], TIM_CHANNEL_ALL);
        s_last_raw[i] = __HAL_TIM_GET_COUNTER(s_htim[i]);
        s_total[i] = 0;
        s_delta[i] = 0;
    }
}

void encoder_update(void)
{
    for (int i = 0; i < 3; i++) {
        uint16_t raw = __HAL_TIM_GET_COUNTER(s_htim[i]);
        int16_t delta = (int16_t)(raw - s_last_raw[i]);
        s_delta[i]  = delta;
        s_total[i] += delta;
        s_last_raw[i] = raw;
    }
}

int32_t encoder_get_count(uint8_t idx) { return (idx < 3) ? s_total[idx] : 0; }
int16_t encoder_get_delta(uint8_t idx) { return (idx < 3) ? s_delta[idx] : 0; }
