#include "bno_port.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

/* Default to 0x4A (shifted left for HAL 8-bit format = 0x94) */
static uint16_t s_bno_i2c_address = (0x4A << 1);
static bool s_bno_detected = false;

bool BNO_Port_Detect(void)
{
    /* Check standard address 0x4A (ADDR / DI0 = GND or default) */
    if (HAL_I2C_IsDeviceReady(&hi2c3, (0x4A << 1), 3, 50) == HAL_OK)
    {
        s_bno_i2c_address = (0x4A << 1);
        s_bno_detected = true;
        return true;
    }

    /* Check alternate address 0x4B (ADDR / DI0 = 3.3V / HIGH) */
    if (HAL_I2C_IsDeviceReady(&hi2c3, (0x4B << 1), 3, 50) == HAL_OK)
    {
        s_bno_i2c_address = (0x4B << 1);
        s_bno_detected = true;
        return true;
    }

    s_bno_detected = false;
    return false;
}

uint8_t BNO_Port_GetAddress(void)
{
    return (uint8_t)(s_bno_i2c_address >> 1);
}

static BNO_PortStatus STM32_BNO_Write(const uint8_t *data,
                                      size_t length)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(
        &hi2c3,
        s_bno_i2c_address,
        (uint8_t *)data,
        length,
        100
    );

    return (status == HAL_OK)
           ? BNO_PORT_OK
           : BNO_PORT_ERROR;
}

static BNO_PortStatus STM32_BNO_Read(uint8_t *data,
                                     size_t length)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Receive(
        &hi2c3,
        s_bno_i2c_address,
        data,
        length,
        100
    );

    return (status == HAL_OK)
           ? BNO_PORT_OK
           : BNO_PORT_ERROR;
}

static void STM32_BNO_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

static void STM32_BNO_Reset(void)
{
    /* Reset line not connected by default */
}

void BNO_Port_Init(BNO_Port *port)
{
    port->write    = STM32_BNO_Write;
    port->read     = STM32_BNO_Read;
    port->delay_ms = STM32_BNO_Delay;
    port->reset    = STM32_BNO_Reset;
}