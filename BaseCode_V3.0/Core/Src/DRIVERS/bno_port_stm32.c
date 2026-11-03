#include <DRIVERS/bno_port.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

/*
 * This must match the configured I2C1 clock in CubeMX.
 * The current project configures I2C1 for 100 kHz.
 */
#define BNO_I2C_BUS_HZ              100000UL
#define BNO_I2C_READ_MARGIN_MS      3UL

/* Default to 0x4A (shifted left for STM32 HAL 8-bit address format = 0x94). */
static uint16_t s_bno_i2c_address = (0x4A << 1);
static bool s_bno_detected = false;

static uint32_t bno_i2c_read_timeout_ms(size_t length)
{
    /* 9 clock periods per byte (8 data bits + ACK), plus a small margin. */
    uint64_t transfer_time_us =
        ((uint64_t)length * 9ULL * 1000000ULL + BNO_I2C_BUS_HZ - 1ULL) /
        BNO_I2C_BUS_HZ;

    return (uint32_t)((transfer_time_us + 999ULL) / 1000ULL) +
           BNO_I2C_READ_MARGIN_MS;
}

bool BNO_Port_Detect(void)
{
    /* Check standard address 0x4A (ADDR / DI0 = GND or default). */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (0x4A << 1), 3, 50) == HAL_OK)
    {
        s_bno_i2c_address = (0x4A << 1);
        s_bno_detected = true;
        return true;
    }

    /* Check alternate address 0x4B (ADDR / DI0 = 3.3V / HIGH). */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (0x4B << 1), 3, 50) == HAL_OK)
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

static BNO_PortStatus STM32_BNO_Write(const uint8_t *data, size_t length)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(
        &hi2c1,
        s_bno_i2c_address,
        (uint8_t *)data,
        (uint16_t)length,
        100U
    );

    return (status == HAL_OK) ? BNO_PORT_OK : BNO_PORT_ERROR;
}

static BNO_PortStatus STM32_BNO_Read(uint8_t *data, size_t length)
{
    HAL_StatusTypeDef status;
    uint32_t timeout_ms = bno_i2c_read_timeout_ms(length);

    status = HAL_I2C_Master_Receive(
        &hi2c1,
        s_bno_i2c_address,
        data,
        (uint16_t)length,
        timeout_ms
    );

    if (status == HAL_OK)
    {
        return BNO_PORT_OK;
    }

    /* BNO085 stretches SCL when polled while no packet is available. */
    if (status == HAL_TIMEOUT)
    {
        return BNO_PORT_TIMEOUT;
    }

    return BNO_PORT_ERROR;
}

static void STM32_BNO_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

static void STM32_BNO_Reset(void)
{
    /* Reset line is not connected in the current hardware configuration. */
}

void BNO_Port_Init(BNO_Port *port)
{
    if (port == NULL)
    {
        return;
    }

    port->write    = STM32_BNO_Write;
    port->read     = STM32_BNO_Read;
    port->delay_ms = STM32_BNO_Delay;
    port->reset    = STM32_BNO_Reset;
}
