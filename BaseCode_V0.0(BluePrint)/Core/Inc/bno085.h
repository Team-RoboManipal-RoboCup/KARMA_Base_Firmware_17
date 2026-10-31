/*
 * bno085.h
 *
 *  Created on: Aug 17, 2026
 *      Author: daivi
 */

#ifndef BNO085_H
#define BNO085_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    float i, j, k, real;     // quaternion components
    float accuracy_rad;
} bno085_rotvec_t;

void    bno085_init(I2C_HandleTypeDef *hi2c,
                     GPIO_TypeDef *rst_port, uint16_t rst_pin,
                     GPIO_TypeDef *int_port, uint16_t int_pin);
void    bno085_hw_reset(void);
uint8_t bno085_probe(uint8_t *sw_major, uint8_t *sw_minor); // 1 = sensor answered
uint8_t bno085_enable_rotation_vector(uint32_t interval_us);
uint8_t bno085_service(bno085_rotvec_t *out);                // 1 = out[] holds a fresh sample

#endif/* INC_BNO085_H_ */
