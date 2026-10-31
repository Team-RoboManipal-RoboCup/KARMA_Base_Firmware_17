#include "hw_encoders.h"

static TIM_HandleTypeDef *s_htims[HW_ENCODER_NUM_CHANNELS];
static uint16_t s_last_raw[HW_ENCODER_NUM_CHANNELS];
static int16_t  s_deltas[HW_ENCODER_NUM_CHANNELS];
static int32_t  s_totals[HW_ENCODER_NUM_CHANNELS];

void hw_encoders_init(TIM_HandleTypeDef *htim0, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2)
{
    s_htims[0] = htim0;
    s_htims[1] = htim1;
    s_htims[2] = htim2;

    for (int i = 0; i < HW_ENCODER_NUM_CHANNELS; i++) {
        if (s_htims[i] != NULL) {
            HAL_TIM_Encoder_Start(s_htims[i], TIM_CHANNEL_ALL);
            s_last_raw[i] = (uint16_t)__HAL_TIM_GET_COUNTER(s_htims[i]);
        } else {
            s_last_raw[i] = 0;
        }
        s_deltas[i] = 0;
        s_totals[i] = 0;
    }
}

void hw_encoders_update(void)
{
    for (int i = 0; i < HW_ENCODER_NUM_CHANNELS; i++) {
        if (s_htims[i] == NULL) {
            continue;
        }

        /* Read raw 16-bit hardware counter */
        uint16_t current_raw = (uint16_t)__HAL_TIM_GET_COUNTER(s_htims[i]);

        /* Unsigned two's complement delta arithmetic handles rollovers across 0 and 65535 seamlessly */
        int16_t delta = (int16_t)(current_raw - s_last_raw[i]);

        s_deltas[i]   = delta;
        s_totals[i]  += delta;
        s_last_raw[i] = current_raw;
    }
}

int16_t hw_encoders_get_delta(uint8_t channel)
{
    if (channel < HW_ENCODER_NUM_CHANNELS) {
        return s_deltas[channel];
    }
    return 0;
}

int32_t hw_encoders_get_total(uint8_t channel)
{
    if (channel < HW_ENCODER_NUM_CHANNELS) {
        return s_totals[channel];
    }
    return 0;
}

void hw_encoders_get_all(hw_encoder_data_t *out_data)
{
    if (!out_data) return;
    for (int i = 0; i < HW_ENCODER_NUM_CHANNELS; i++) {
        out_data->deltas[i]       = s_deltas[i];
        out_data->total_counts[i] = s_totals[i];
    }
}
