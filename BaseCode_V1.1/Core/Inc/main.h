/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define FIRMWARE_VERSION        "1.1"
#define TELEMETRY_INTERVAL_MS   20    /* 50 Hz JSON telemetry rate */
#define MOTOR_WATCHDOG_TIMEOUT  1500  /* Stop motors if no command within 1500 ms (0 to disable) */

/* FrSky SmartPort current sensor (PA7, bit-banged) */
#define SPORT_POLL_OFFSET_MS    8     /* Poll this many ms after each telemetry frame */
#define SPORT_STALE_MS          1000  /* Report sensor offline ("cs":0) if no current frame within this time */
#ifndef SPORT_POLL_PERIOD_MS
#define SPORT_POLL_PERIOD_MS    12U   /* Free-running 12 ms polling interval */
#endif
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* Core Robot Functions */
void robot_init(void);
void robot_loop(void);
void robot_process_rx(uint8_t *buf, uint32_t len);

/* Direct Control Helpers */
void robot_set_motor(uint8_t motor_id, int16_t speed);
void robot_set_all_motors(int16_t speed);
void robot_stop(void);
int16_t robot_get_motor_pwm(uint8_t motor_id);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_PWM_Pin GPIO_PIN_0
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_1
#define M2_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_2
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_3
#define M4_PWM_GPIO_Port GPIOA
#define CUR3_Pin GPIO_PIN_7
#define CUR3_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOB
#define DIR4_Pin GPIO_PIN_12
#define DIR4_GPIO_Port GPIOB
#define ENC3_A_TIM2_CH1_Pin GPIO_PIN_15
#define ENC3_A_TIM2_CH1_GPIO_Port GPIOA
#define ENC3_B_TIM2_CH2_Pin GPIO_PIN_3
#define ENC3_B_TIM2_CH2_GPIO_Port GPIOB
#define ENC2_A_TIM3_CH1_Pin GPIO_PIN_4
#define ENC2_A_TIM3_CH1_GPIO_Port GPIOB
#define ENC2_B_TIM3_CH2_Pin GPIO_PIN_5
#define ENC2_B_TIM3_CH2_GPIO_Port GPIOB
#define ENC1_A_TIM4_CH1_Pin GPIO_PIN_6
#define ENC1_A_TIM4_CH1_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
