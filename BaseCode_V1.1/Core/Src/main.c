/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Ultra-Clean Version)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "bno085_driver.h"
#include "bno_port.h"
#include "encoder.h"
#include "motor.h"
#include "sport.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
/* BNO085 instances */
static BNO085 s_bno;
static BNO_Port s_bno_port;
static bool s_bno_ready = false;
static uint32_t s_last_feature_retry = 0;

/* State variables */
static uint32_t s_last_telemetry_tick = 0;
static uint32_t s_last_cmd_tick = 0;
static bool s_motors_active = false;
static int16_t s_motor_pwm[4] = {0, 0, 0, 0};

/* SmartPort current sensor state */
static float s_current_amps = 0.0f;
static uint32_t s_last_sport_ok_tick = 0;
static uint32_t s_last_sport_poll_tick = 0;
static bool s_sport_ever_ok = false;

/* USB Buffers */
static char s_tx_buf[384];
static char s_ack_buf[256];
static volatile bool s_pending_ack = false;
static char s_rx_line[128];
static uint8_t s_rx_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000); /* Allow USB CDC and peripherals to initialize */

  /* Initialize autonomous robot subsystems (Encoders, MD30C Motors, BNO085) */
  robot_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Service odometry, IMU, motor safety, and USB CDC JSON telemetry */
    robot_loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CUR3_GPIO_Port, CUR3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR1_Pin|DIR2_Pin|DIR3_Pin|DIR4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CUR3_Pin */
  GPIO_InitStruct.Pin = CUR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CUR3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin DIR3_Pin DIR4_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin|DIR3_Pin|DIR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Helper to parse an integer following a key in a JSON-like string */
static bool parse_json_int(const char *str, const char *key, int *out_val) {
  const char *p = strstr(str, key);
  if (!p)
    return false;

  p += strlen(key);
  while (*p && (*p == ' ' || *p == ':' || *p == '"'))
    p++;

  if (*p == '-' || (*p >= '0' && *p <= '9')) {
    *out_val = atoi(p);
    return true;
  }
  return false;
}

void robot_set_motor(uint8_t motor_id, int16_t speed) {
  if (motor_id > 3)
    return;
  if (speed > MOTOR_MAX_PWM_LIMIT)
    speed = MOTOR_MAX_PWM_LIMIT;
  if (speed < -MOTOR_MAX_PWM_LIMIT)
    speed = -MOTOR_MAX_PWM_LIMIT;
  s_motor_pwm[motor_id] = speed;
  motor_set(motor_id, speed);
  if (speed != 0)
    s_motors_active = true;
}

void robot_set_all_motors(int16_t speed) {
  for (int i = 0; i < 4; i++) {
    robot_set_motor(i, speed);
  }
}

void robot_stop(void) {
  for (int i = 0; i < 4; i++) {
    s_motor_pwm[i] = 0;
  }
  motor_stop_all();
  s_motors_active = false;
}

int16_t robot_get_motor_pwm(uint8_t motor_id) {
  if (motor_id < 4)
    return s_motor_pwm[motor_id];
  return 0;
}

/*
 * Parses incoming commands over USB CDC.
 * Supports:
 *   1. Individual JSON keys: {"m1": 500, "m2": 500, "m3": -500, "m4": -500}
 *   2. Array format:         {"m": [500, 500, -500, -500]} or {"pwm": [500, 500, -500, -500]}
 *   3. Broadcast speed:      {"speed": 300} or {"pwm": 300}
 *   4. Emergency stop:       {"cmd": "stop"} or "stop"
 *   5. SmartPort diagnostics: "sport", "diag", "poll", or {"cmd":"sport"}
 *   6. Raw number input:     "50" (sets all motors to 50% / 500 speed)
 */
static void parse_command(const char *cmd) {
  s_last_cmd_tick = HAL_GetTick();

  /* Emergency stop check */
  if (strstr(cmd, "stop") != NULL) {
    robot_stop();
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"cmd\":\"stop\",\"pwm\":[0,0,0,0]}\r\n");
    s_pending_ack = true;
    return;
  }

  /* SmartPort diagnostics / polling command: "sport", "diag", "poll" */
  if (strstr(cmd, "sport") != NULL || strstr(cmd, "diag") != NULL || strstr(cmd, "poll") != NULL) {
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"sport\":{\"wire\":\"%s\",\"clk\":%lu,\"bit\":%lu,\"id\":%u,\"lock\":%u,"
             "\"rx\":%u,\"crc\":%d,\"app\":%u,\"raw\":%lu,\"cur\":%.2f,\"vb\":%.2f,"
             "\"reads\":%lu,\"polls\":%lu,\"frame\":[%u,%u,%u,%u,%u,%u,%u,%u]}}\r\n",
             g_sport_debug.wire_state_idle ? "HIGH" : "LOW",
             (unsigned long)g_sport_debug.clock_hz,
             (unsigned long)g_sport_debug.bit_cycles,
             g_sport_debug.last_poll_id, g_sport_debug.locked_poll_id,
             g_sport_debug.bytes_received, g_sport_debug.last_crc_ok ? 1 : 0,
             g_sport_debug.last_app_id, (unsigned long)g_sport_debug.last_raw_val,
             g_sport_debug.current_amps, g_sport_debug.voltage_volts,
             (unsigned long)g_sport_debug.successful_reads,
             (unsigned long)g_sport_debug.total_polls,
             g_sport_debug.raw_frame[0], g_sport_debug.raw_frame[1],
             g_sport_debug.raw_frame[2], g_sport_debug.raw_frame[3],
             g_sport_debug.raw_frame[4], g_sport_debug.raw_frame[5],
             g_sport_debug.raw_frame[6], g_sport_debug.raw_frame[7]);
    s_pending_ack = true;
    return;
  }

  /* 1. Check for individual motor keys "m1", "m2", "m3", "m4" */
  int val;
  bool found_individual = false;
  if (parse_json_int(cmd, "\"m1\"", &val)) {
    robot_set_motor(0, (int16_t)val);
    found_individual = true;
  }
  if (parse_json_int(cmd, "\"m2\"", &val)) {
    robot_set_motor(1, (int16_t)val);
    found_individual = true;
  }
  if (parse_json_int(cmd, "\"m3\"", &val)) {
    robot_set_motor(2, (int16_t)val);
    found_individual = true;
  }
  if (parse_json_int(cmd, "\"m4\"", &val)) {
    robot_set_motor(3, (int16_t)val);
    found_individual = true;
  }
  if (found_individual) {
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
             s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
    s_pending_ack = true;
    return;
  }

  /* 2. Check for array format: {"m": [...]} or {"pwm": [...]} */
  const char *p = strstr(cmd, "\"m\":");
  if (!p)
    p = strstr(cmd, "\"pwm\":");
  if (p) {
    const char *bracket = strchr(p, '[');
    if (bracket) {
      bracket++;
      for (int i = 0; i < 4; i++) {
        while (*bracket == ' ' || *bracket == ',')
          bracket++;
        if (!*bracket || *bracket == ']')
          break;
        int spd = atoi(bracket);
        robot_set_motor(i, (int16_t)spd);
        while (*bracket && *bracket != ',' && *bracket != ']')
          bracket++;
        if (*bracket == ',')
          bracket++;
      }
      snprintf(s_ack_buf, sizeof(s_ack_buf),
               "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
               s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
      s_pending_ack = true;
      return;
    } else {
      /* Single broadcast value: {"pwm": 500} */
      const char *col = strchr(p, ':');
      if (col && parse_json_int(col - 5, "\"pwm\"", &val)) {
        robot_set_all_motors((int16_t)val);
        snprintf(s_ack_buf, sizeof(s_ack_buf),
                 "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n",
                 s_motor_pwm[0], s_motor_pwm[1], s_motor_pwm[2],
                 s_motor_pwm[3]);
        s_pending_ack = true;
        return;
      }
    }
  }

  /* 3. Check for broadcast speed: {"speed": 500} */
  if (parse_json_int(cmd, "\"speed\"", &val)) {
    robot_set_all_motors((int16_t)val);
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
             s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
    s_pending_ack = true;
    return;
  }

  /* 4. Fallback: Raw numeric input (e.g. "50" or "-80" from serial terminal) */
  const char *c = cmd;
  while (*c == ' ' || *c == '\t')
    c++;
  if (*c == '-' || (*c >= '0' && *c <= '9')) {
    int raw_val = atoi(c);
    /* If sent as percentage -100..100, scale to -1000..1000 */
    if (raw_val >= -100 && raw_val <= 100) {
      raw_val *= 10;
    }
    robot_set_all_motors((int16_t)raw_val);
    snprintf(s_ack_buf, sizeof(s_ack_buf),
             "{\"status\":\"OK\",\"pwm\":[%d,%d,%d,%d]}\r\n", s_motor_pwm[0],
             s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);
    s_pending_ack = true;
    return;
  }

  /* Unrecognized command */
  snprintf(s_ack_buf, sizeof(s_ack_buf),
           "{\"status\":\"ERR\",\"msg\":\"unknown cmd\"}\r\n");
  s_pending_ack = true;
}

void robot_process_rx(uint8_t *buf, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    char c = (char)buf[i];

    if (c == '\r' || c == '\n') {
      if (s_rx_idx > 0) {
        s_rx_line[s_rx_idx] = '\0';
        parse_command(s_rx_line);
        s_rx_idx = 0;
      }
    } else if (s_rx_idx < (sizeof(s_rx_line) - 1)) {
      s_rx_line[s_rx_idx++] = c;
    }
  }
}

void robot_init(void) {
  /* 1. Initialize 3 Dead Wheel Encoders (TIM4, TIM3, TIM2) */
  encoder_init(&htim4, &htim3, &htim2);

  /* 2. Initialize 4x MD30C Motor Drivers (PWM on TIM5 CH1..CH4, DIR on GPIOB) */
  motor_config_t my_motors[4] = {
      {TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin}, /* M1: PA0 (TIM5_CH1), DIR: PB0 */
      {TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin}, /* M2: PA1 (TIM5_CH2), DIR: PB1 */
      {TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin}, /* M3: PA2 (TIM5_CH3), DIR: PB2 */
      {TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin}  /* M4: PA3 (TIM5_CH4), DIR: PB12 */
  };
  motor_init(&htim5, my_motors);

  /* 3. Initialize BNO085 IMU */
  BNO_Port_Init(&s_bno_port);
  if (BNO_Port_Detect()) {
    if (BNO085_Init(&s_bno, &s_bno_port) == BNO_PORT_OK) {
      /* Drain initial startup and advertisement SHTP packets from BNO boot */
      for (int i = 0; i < 15; i++) {
        BNO085_Update(&s_bno);
        HAL_Delay(10);
      }

      /* Enable Game Rotation Vector (50Hz / 20000us) for drift-free yaw */
      BNO085_EnableGameRotation(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable standard Rotation Vector as fallback */
      BNO085_EnableRotation(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable Linear Acceleration (50Hz / 20000us) for gravity-free x, y */
      BNO085_EnableLinearAccel(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable standard Acceleration as fallback */
      BNO085_EnableAccel(&s_bno, 20000);
      HAL_Delay(20);
      /* Enable Calibrated Gyroscope (50Hz / 20000us) for angular velocity x, y */
      BNO085_EnableGyro(&s_bno, 20000);
      HAL_Delay(20);

      s_bno_ready = true;
    }
  }

  /* 4. Initialize FrSky SmartPort current sensor on PA7 (DWT-timed bit-bang). */
  SPort_Init();

  s_last_telemetry_tick = HAL_GetTick();
  s_last_cmd_tick = HAL_GetTick();
  s_last_sport_poll_tick = HAL_GetTick();
}

void robot_loop(void) {
  /* 1. Process pending BNO085 SHTP packets */
  if (s_bno_ready) {
    for (int i = 0; i < 8; i++) {
      if (BNO085_Update(&s_bno) != BNO_PORT_OK)
        break;
    }

    /* If no sensor reports have arrived yet, re-send SetFeature requests every 250ms */
    if (s_bno.diag.rx_reports == 0 &&
        (HAL_GetTick() - s_last_feature_retry >= 250)) {
      s_last_feature_retry = HAL_GetTick();
      BNO085_EnableGameRotation(&s_bno, 20000);
      BNO085_EnableRotation(&s_bno, 20000);
      BNO085_EnableLinearAccel(&s_bno, 20000);
      BNO085_EnableAccel(&s_bno, 20000);
      BNO085_EnableGyro(&s_bno, 20000);
    }
  }

  /* Service immediate command acknowledgment */
  if (s_pending_ack) {
    if (CDC_Transmit_FS((uint8_t *)s_ack_buf, (uint16_t)strlen(s_ack_buf)) ==
        USBD_OK) {
      s_pending_ack = false;
    }
  }

  /* 2. FrSky SmartPort current poll cycle (every ~12 ms) */
  if (HAL_GetTick() - s_last_sport_poll_tick >= SPORT_POLL_PERIOD_MS) {
    s_last_sport_poll_tick = HAL_GetTick();
    float amps;
    if (SPort_PollCurrent(&amps)) {
      s_current_amps = amps;
      s_last_sport_ok_tick = HAL_GetTick();
      s_sport_ever_ok = true;
    }
  }

  /* 3. Periodic JSON Telemetry Output (50 Hz / 20 ms) */
  if (!s_pending_ack &&
      (HAL_GetTick() - s_last_telemetry_tick >= TELEMETRY_INTERVAL_MS)) {
    s_last_telemetry_tick = HAL_GetTick();
    uint32_t t_ms = s_last_telemetry_tick;

    /* Update 3 dead wheel encoder counts */
    encoder_update();
    int32_t e1 = encoder_get_count(0);
    int32_t e2 = encoder_get_count(1);
    int32_t e3 = encoder_get_count(2);

    /* Get BNO085 Heading (Yaw in degrees) */
    BNO_Quaternion q;
    float yaw = 0.0f;
    if (BNO085_GetGameRotation(&s_bno, &q) || BNO085_GetRotation(&s_bno, &q)) {
      yaw = BNO085_GetEulerYaw(&q);
    }

    /* Get BNO085 Acceleration (x, y in m/s^2) - No Z axis */
    BNO_Vector3 accel = {0.0f, 0.0f, 0.0f};
    if (!BNO085_GetLinearAccel(&s_bno, &accel)) {
      BNO085_GetAccel(&s_bno, &accel);
    }

    /* Get BNO085 Angular Velocity (x, y in rad/s) - No Z axis */
    BNO_Vector3 gyro = {0.0f, 0.0f, 0.0f};
    BNO085_GetGyro(&s_bno, &gyro);

    /* SmartPort current sensor status (stale after SPORT_STALE_MS) */
    bool sport_ok = s_sport_ever_ok &&
                    ((t_ms - s_last_sport_ok_tick) < SPORT_STALE_MS);

    /* Abbreviated JSON telemetry */
    int len = snprintf(
          s_tx_buf, sizeof(s_tx_buf),
          "{\"t\":%lu,\"yaw\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"wx\":%.2f,\"wy\":%.2f,"
          "\"e1\":%ld,\"e2\":%ld,\"e3\":%ld,\"ok\":%d,\"pwm\":[%d,%d,%d,%d],"
          "\"cur\":%.2f,\"vb\":%.2f,\"cs\":%d}\r\n",
          (unsigned long)t_ms, yaw, accel.x, accel.y, gyro.x, gyro.y,
          (long)e1, (long)e2, (long)e3,
          s_bno_ready ? 1 : 0, s_motor_pwm[0], s_motor_pwm[1], s_motor_pwm[2],
          s_motor_pwm[3],
          s_current_amps, g_sport_debug.voltage_volts, sport_ok ? 1 : 0);

    if (len > 0) {
      CDC_Transmit_FS((uint8_t *)s_tx_buf, (uint16_t)len);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
