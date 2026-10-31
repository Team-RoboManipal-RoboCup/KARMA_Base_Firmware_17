/*
 * encoder.h
 *
 *  Created on: Aug 18, 2026
 *      Author: daivi
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

void    encoder_init(TIM_HandleTypeDef *h0, TIM_HandleTypeDef *h1, TIM_HandleTypeDef *h2);
void    encoder_update(void);
int32_t encoder_get_count(uint8_t idx);
int16_t encoder_get_delta(uint8_t idx);

#endif/* INC_ENCODER_H_ */
