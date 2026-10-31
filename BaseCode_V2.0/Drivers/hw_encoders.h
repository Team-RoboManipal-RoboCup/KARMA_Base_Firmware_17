#ifndef DRIVERS_HW_ENCODERS_H
#define DRIVERS_HW_ENCODERS_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HW_ENCODER_NUM_CHANNELS 3

typedef struct {
    int32_t  total_counts[HW_ENCODER_NUM_CHANNELS];
    int16_t  deltas[HW_ENCODER_NUM_CHANNELS];
} hw_encoder_data_t;

/**
 * @brief Initialize 3 hardware encoder timer channels (TIM4, TIM3, TIM2).
 */
void hw_encoders_init(TIM_HandleTypeDef *htim0, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2);

/**
 * @brief Update encoder readings using unsigned two's complement arithmetic across 16-bit boundaries.
 */
void hw_encoders_update(void);

/**
 * @brief Get instantaneous delta count for channel (0, 1, or 2).
 */
int16_t hw_encoders_get_delta(uint8_t channel);

/**
 * @brief Get total accumulated count for channel (0, 1, or 2).
 */
int32_t hw_encoders_get_total(uint8_t channel);

/**
 * @brief Capture a snapshot of all encoder channels.
 */
void hw_encoders_get_all(hw_encoder_data_t *out_data);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_HW_ENCODERS_H */
