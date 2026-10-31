#include "i2c_recovery.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

static void delay_us(uint32_t us)
{
    /* At 96 MHz Cortex-M4, ~24 loop cycles ~= 1 us */
    uint32_t count = us * (SystemCoreClock / 4000000);
    while (count--) {
        __NOP();
    }
}

bool i2c_bus_recovery_perform(void)
{
    /* 1. De-initialize I2C3 peripheral hardware */
    HAL_I2C_DeInit(&hi2c3);

    /* 2. Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 3. Configure PA8 (SCL) and PB8 (SDA) as Open-Drain GPIO outputs with pullups */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); /* PA8 = SCL */

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); /* PB8 = SDA */

    /* Initially let both lines float high */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10);

    /* 4. Clock SCL 9 times to force stuck slave to release SDA */
    for (int i = 0; i < 9; i++) {
        /* Check if SDA is released already */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) {
            /* SDA is high; slave has released */
            break;
        }

        /* Pulse SCL low then high */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(5);
    }

    /* 5. Generate manual STOP condition: SDA low -> SCL high -> SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); /* SDA low */
    delay_us(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   /* SCL high */
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   /* SDA rising while SCL high = STOP */
    delay_us(10);

    bool sda_released = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET);

    /* 6. Restore PA8 and PB8 to alternate function AF4 (I2C3) */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Alternate = GPIO_AF9_I2C3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 7. Re-initialize I2C3 hardware */
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c3);

    return sda_released;
}
