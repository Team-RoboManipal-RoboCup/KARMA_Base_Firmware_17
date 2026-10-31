/*
 * bno085.c
 *
 *  Created on: Aug 17, 2026
 *      Author: daivi
 */
#include "bno085.h"
#include <string.h>

#define BNO085_I2C_ADDR   0x4A     /* 7-bit; SA0 pin low */
#define I2C_TIMEOUT_MS    50

#define CHAN_COMMAND      0
#define CHAN_EXECUTABLE   1
#define CHAN_CONTROL      2
#define CHAN_REPORTS      3
#define CHAN_WAKE_REPORTS 4

#define RID_SET_FEATURE_CMD   0xFD
#define RID_PRODUCT_ID_REQ    0xF9
#define RID_PRODUCT_ID_RESP   0xF8
#define RID_BASE_TIMESTAMP    0xFB
#define RID_ROTATION_VECTOR   0x05

static I2C_HandleTypeDef *s_hi2c;
static GPIO_TypeDef *s_rst_port, *s_int_port;
static uint16_t s_rst_pin, s_int_pin;
static uint8_t  s_seq[6];
static uint8_t  s_rxbuf[64];   /* bump if you enable multiple simultaneous features */

static uint8_t bno085_int_asserted(void)
{
    return HAL_GPIO_ReadPin(s_int_port, s_int_pin) == GPIO_PIN_RESET;
}

static uint8_t shtp_write(uint8_t chan, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[4 + 32];
    if (len > 32) return 0;
    buf[0] = (4 + len) & 0xFF;
    buf[1] = ((4 + len) >> 8) & 0x7F;   /* bit7 = continuation flag, unused here */
    buf[2] = chan;
    buf[3] = s_seq[chan]++;
    memcpy(&buf[4], payload, len);
    return HAL_I2C_Master_Transmit(s_hi2c, BNO085_I2C_ADDR << 1, buf, 4 + len,
                                    I2C_TIMEOUT_MS) == HAL_OK;
}

/* Only issues an I2C transfer when INT is already asserted -- this is what
 * prevents the indefinite clock-stretch failure mode. Returns cargo length
 * (payload after the 4-byte SHTP header), 0 if nothing pending. */
static uint16_t shtp_read(void)
{
    uint8_t hdr[4];
    if (!bno085_int_asserted()) return 0;

    if (HAL_I2C_Master_Receive(s_hi2c, BNO085_I2C_ADDR << 1, hdr, 4, I2C_TIMEOUT_MS) != HAL_OK)
        return 0;

    uint16_t total_len = ((hdr[1] & 0x7F) << 8) | hdr[0];
    if (total_len <= 4) return 0;
    if (total_len > sizeof(s_rxbuf)) total_len = sizeof(s_rxbuf);  /* clamp: bring-up only */

    if (HAL_I2C_Master_Receive(s_hi2c, BNO085_I2C_ADDR << 1, s_rxbuf, total_len,
                                I2C_TIMEOUT_MS) != HAL_OK)
        return 0;

    return total_len - 4;
}

void bno085_init(I2C_HandleTypeDef *hi2c,
                  GPIO_TypeDef *rst_port, uint16_t rst_pin,
                  GPIO_TypeDef *int_port, uint16_t int_pin)
{
    s_hi2c = hi2c;
    s_rst_port = rst_port; s_rst_pin = rst_pin;
    s_int_port = int_port; s_int_pin = int_pin;
    memset(s_seq, 0, sizeof(s_seq));
}

void bno085_hw_reset(void)
{
    HAL_GPIO_WritePin(s_rst_port, s_rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(s_rst_port, s_rst_pin, GPIO_PIN_SET);

    /* Time-boxed drain of the boot-time advertisement/reset packets.
     * Not response-matched -- fine for bring-up, revisit if this
     * becomes production firmware (should key off the executable
     * "reset complete" packet on CHAN_EXECUTABLE instead). */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 300) shtp_read();
}

uint8_t bno085_probe(uint8_t *sw_major, uint8_t *sw_minor)
{
    uint8_t req[2] = { RID_PRODUCT_ID_REQ, 0x00 };
    if (!shtp_write(CHAN_CONTROL, req, sizeof(req))) return 0;

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 200) {
        uint16_t n = shtp_read();
        if (n >= 6 && s_rxbuf[4] == RID_PRODUCT_ID_RESP) {
            /* Offsets below per SH-2 Reference Manual Product ID Response
             * layout -- cross-check against the datasheet if these print
             * obviously wrong values; this path is diagnostic-only. */
            if (sw_major) *sw_major = s_rxbuf[6];
            if (sw_minor) *sw_minor = s_rxbuf[7];
            return 1;
        }
    }
    return 0;
}

uint8_t bno085_enable_rotation_vector(uint32_t interval_us)
{
    uint8_t p[17] = {0};
    p[0] = RID_SET_FEATURE_CMD;
    p[1] = RID_ROTATION_VECTOR;
    p[5] = (interval_us)       & 0xFF;
    p[6] = (interval_us >> 8)  & 0xFF;
    p[7] = (interval_us >> 16) & 0xFF;
    p[8] = (interval_us >> 24) & 0xFF;
    return shtp_write(CHAN_CONTROL, p, sizeof(p));
}

uint8_t bno085_service(bno085_rotvec_t *out)
{
    uint16_t n = shtp_read();
    if (n == 0) return 0;

    uint8_t *cargo = &s_rxbuf[4];
    uint16_t i = 0;
    uint8_t found = 0;

    while (i + 1 < n) {
        uint8_t report_id = cargo[i];
        if (report_id == RID_BASE_TIMESTAMP) { i += 5; continue; }
        if (report_id == RID_ROTATION_VECTOR && i + 14 <= n) {
            int16_t qi  = (int16_t)(cargo[i+5]  << 8 | cargo[i+4]);
            int16_t qj  = (int16_t)(cargo[i+7]  << 8 | cargo[i+6]);
            int16_t qk  = (int16_t)(cargo[i+9]  << 8 | cargo[i+8]);
            int16_t qr  = (int16_t)(cargo[i+11] << 8 | cargo[i+10]);
            int16_t acc = (int16_t)(cargo[i+13] << 8 | cargo[i+12]);
            out->i    = qi  / 16384.0f;   // Q14
            out->j    = qj  / 16384.0f;
            out->k    = qk  / 16384.0f;
            out->real = qr  / 16384.0f;
            out->accuracy_rad = acc / 4096.0f; // Q12
            found = 1;
            i += 14;
            continue;
        }
        break;  /* unrecognized report -- bail rather than risk misparsing the rest */
    }
    return found;
}

