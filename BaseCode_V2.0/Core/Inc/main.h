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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

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
