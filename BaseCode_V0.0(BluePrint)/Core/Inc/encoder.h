/*
 * encoder.h
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void    encoder_init(TIM_HandleTypeDef *h0, TIM_HandleTypeDef *h1, TIM_HandleTypeDef *h2);
void    encoder_update(void);               // call once per control-loop tick
int32_t encoder_get_count(uint8_t idx);      // idx 0..2, cumulative signed count since init
int16_t encoder_get_delta(uint8_t idx);      // counts since the last encoder_update()

#endif/* INC_ENCODER_H_ */
