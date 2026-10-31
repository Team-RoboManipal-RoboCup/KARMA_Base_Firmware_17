/*
 * sport.c
 *
 *  Created on: Oct 26, 2026
 *      Author: daivi
 */
/**
  ******************************************************************************
  * @file           : sport.c
  * @brief          : FrSky SmartPort (S.Port) software bit-bang driver for STM32F411
  ******************************************************************************
  */

#include "sport.h"
#include <stdio.h>
#include <string.h>

/*
 * FrSky SmartPort operates at 57600 baud, 8N1, half-duplex single wire, inverted logic.
 *
 * Wire States:
 * - Idle state: LOW (0V)
 * - Start bit: HIGH (3.3V)
 * - Data bit 1: LOW (0V), Data bit 0: HIGH (3.3V)
 * - Stop bit: LOW (0V)
 *
 * STM32F411 PA7 GPIO Register Mapping:
 * - MODER bits [15:14]: 00 = Input, 01 = Output (General Purpose)
 * - PUPDR bits [15:14]: 00 = No pull, 01 = Pull-Up, 10 = Pull-Down
 * - OSPEEDR bits [15:14]: 11 = Very High Speed (50 - 100 MHz)
 * - OTYPER bit 7: 0 = Push-Pull
 * - BSRR: Bit 7 = Set HIGH, Bit 23 (7+16) = Reset LOW
 * - IDR: Bit 7 = Read Pin
 */

#define PA7_SET_OUTPUT() do { \
    GPIOA->MODER = (GPIOA->MODER & ~(0x3U << 14)) | (0x1U << 14); \
    GPIOA->PUPDR &= ~(0x3U << 14); \
    GPIOA->OTYPER &= ~(1U << 7); \
    GPIOA->OSPEEDR |= (0x3U << 14); \
} while(0)

#define PA7_SET_INPUT() do { \
    GPIOA->MODER &= ~(0x3U << 14); \
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(0x3U << 14)) | (0x2U << 14); /* Pull-Down: 10b */ \
    GPIOA->BSRR = ((uint32_t)GPIO_PIN_7 << 16U); /* Clear output register latch */ \
} while(0)

#define PA7_HIGH() (GPIOA->BSRR = GPIO_PIN_7)
#define PA7_LOW()  (GPIOA->BSRR = ((uint32_t)GPIO_PIN_7 << 16U))
#define PA7_READ() ((GPIOA->IDR & GPIO_PIN_7) != 0)

static uint32_t bitCycles;
static bool s_rx_frame_active = false;

/*
 * Full 28 FrSky SmartPort physical sensor IDs with embedded CRC-3.
 * Prioritize Current sensors (0x22 = FAS-40/100, 0xE4 = FCS-40A/150A/RPM).
 */
static const uint8_t pollIds[] = {
    0x22, // ID 2: FAS Current / Voltage (Default for FAS-40/100)
    0xE4, // ID 4: FCS Current / RPM Sensor (Default for FCS-40A/150A)
    0xA1, // ID 1: FLVSS LiPo Voltage
    0x00, // ID 0: Vario
    0x83, // ID 3: GPS
    0x45, // ID 5: SP2UART
    0xC6, // ID 6: SP2UART
    0x67, // ID 7: SP2UART
    0x48, // ID 8
    0x98, // ID 24
    0xE9, 0x6A, 0xCB, 0xAC, 0x0D, 0x8E, 0x2F,
    0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16,
    0xB7, 0x39, 0xBA, 0x1B
};
static uint8_t pollIdx = 0;

/* Once a sensor answers with a current frame, keep polling only that ID.
 * After SPORT_LOCK_MAX_MISSES consecutive polls without a valid frame, go back to scanning. */
#define SPORT_LOCK_NONE        0xFFU
#define SPORT_LOCK_MAX_MISSES  10U
static uint8_t lockedIdx = SPORT_LOCK_NONE;
static uint8_t lockMisses = 0;

SPort_DebugInfo_t g_sport_debug = {0};

static inline void waitUntilCycle(uint32_t startCycle, uint32_t targetOffset) {
    while ((DWT->CYCCNT - startCycle) < targetOffset);
}

void SPort_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA7 for Very High Speed, Push-Pull */
    GPIOA->OSPEEDR |= (0x3U << 14);
    GPIOA->OTYPER &= ~(1U << 7);

    /* Enable DWT Cycle Counter on Cortex-M4 (with CoreSight unlock) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *(volatile uint32_t *)0xE0001FB0 = 0xC5ACCE55; // CoreSight Lock Access Register
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    SystemCoreClockUpdate();
    bitCycles = SystemCoreClock / 57600UL;

    g_sport_debug.clock_hz = SystemCoreClock;
    g_sport_debug.bit_cycles = bitCycles;
    g_sport_debug.locked_poll_id = SPORT_LOCK_NONE;

    /* Start in Input mode with Pull-Down (idle wire state is LOW) */
    PA7_SET_INPUT();
}

void SPort_SendPollHeader(uint8_t pollId) {
	uint32_t irq_state = __get_PRIMASK();

    PA7_SET_OUTPUT();
    __disable_irq();

    // 1. Transmit 0x7E
    uint32_t start = DWT->CYCCNT;
    uint32_t target = bitCycles;
    PA7_HIGH(); // Inverted Start Bit (HIGH)
    waitUntilCycle(start, target);

    uint8_t b = 0x7E;
    for (uint8_t i = 0; i < 8; i++) {
        target += bitCycles;
        if (b & 1) PA7_LOW();
        else       PA7_HIGH();
        b >>= 1;
        waitUntilCycle(start, target);
    }
    target += bitCycles;
    PA7_LOW(); // Inverted Stop Bit (LOW)
    waitUntilCycle(start, target);

    // 2. Transmit Physical ID byte immediately
    start = DWT->CYCCNT;
    target = bitCycles;
    PA7_HIGH(); // Inverted Start Bit (HIGH)
    waitUntilCycle(start, target);

    b = pollId;
    for (uint8_t i = 0; i < 8; i++) {
        target += bitCycles;
        if (b & 1) PA7_LOW();
        else       PA7_HIGH();
        b >>= 1;
        waitUntilCycle(start, target);
    }
    target += bitCycles;
    PA7_LOW(); // Inverted Stop Bit (LOW)
    waitUntilCycle(start, target);

    /* Release the SmartPort bus before servicing pending interrupts. */
    PA7_SET_INPUT();

    if (irq_state == 0U) {
        __enable_irq();
    }
}

bool SPort_ReadByte(uint8_t *out, uint32_t timeoutUs) {
	uint32_t irq_state = __get_PRIMASK();
    uint32_t timeoutCycles = timeoutUs * (SystemCoreClock / 1000000UL);
    uint32_t waitStart = DWT->CYCCNT;

    // Wait for start bit (wire goes HIGH)
    while (!PA7_READ()) {
        if ((DWT->CYCCNT - waitStart) > timeoutCycles) return false;
    }

    __disable_irq();
    uint32_t start = DWT->CYCCNT;
    uint32_t target = bitCycles + (bitCycles / 2); // Center of bit 0
    waitUntilCycle(start, target);

    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (!PA7_READ()) { // Inverted: LOW on wire = Logical 1
            val |= (1 << i);
        }
        target += bitCycles;
        waitUntilCycle(start, target);
    }
    // At i=7, target advanced to 9.5 bitCycles (center of stop bit).
    // Stop bit (LOW) is currently active. Return now so caller is ready for next byte!
    *out = val;

    /* Preserve the interrupt mask during a complete frame reception. */
    if (!s_rx_frame_active && irq_state == 0U) {
        __enable_irq();
    }

    return true;
}

bool SPort_PollCurrent(float *currentAmps) {
    uint8_t thisIdx;
    if (lockedIdx != SPORT_LOCK_NONE) {
        thisIdx = lockedIdx;
    } else {
        thisIdx = pollIdx;
        pollIdx = (pollIdx + 1) % (sizeof(pollIds) / sizeof(pollIds[0]));
    }
    uint8_t currentPollId = pollIds[thisIdx];

    g_sport_debug.last_poll_id = currentPollId;
    g_sport_debug.wire_state_idle = PA7_READ();
    g_sport_debug.total_polls++;

    SPort_SendPollHeader(currentPollId);

    uint8_t frame[8];
    uint8_t count = 0;
    bool stuffed = false;
    bool frameOk = false;

    // First byte timeout: up to 5000 us (5 ms) for sensor turnaround
    // Subsequent bytes timeout: 2000 us (2 ms) inter-byte
    uint32_t rx_irq_state = __get_PRIMASK();
    s_rx_frame_active = true;
    while (count < 8) {
        uint8_t b;
        uint32_t timeout = (count == 0) ? 5000 : 2000;
        if (!SPort_ReadByte(&b, timeout)) break;

        if (b == 0x7D) { stuffed = true; continue; }
        if (stuffed)   { b ^= 0x20; stuffed = false; }
        frame[count++] = b;
    }
    s_rx_frame_active = false;

    /* Restore the original interrupt state after reception ends. */
    if (rx_irq_state == 0U && __get_PRIMASK() != 0U) {
        __enable_irq();
    }
    g_sport_debug.bytes_received = count;
    for (uint8_t i = 0; i < 8; i++) {
        g_sport_debug.raw_frame[i] = (i < count) ? frame[i] : 0;
    }

    if (count == 8 && frame[0] == 0x10) {
        uint16_t sum = 0;
        for (int i = 0; i < 8; i++) {
            sum += frame[i];
            sum += (sum >> 8);
            sum &= 0x00FF;
        }
        if (sum == 0xFF) {
            g_sport_debug.last_crc_ok = true;
            frameOk = true;
            lockMisses = 0;
            uint16_t appId = frame[1] | ((uint16_t)frame[2] << 8);
            uint32_t rawVal = (uint32_t)frame[3] |
                              ((uint32_t)frame[4] << 8) |
                              ((uint32_t)frame[5] << 16) |
                              ((uint32_t)frame[6] << 24);

            g_sport_debug.last_app_id = appId;
            g_sport_debug.last_raw_val = rawVal;

            // AppID 0x0200..0x020F: Current (0.1 A units)
            if ((appId & 0xFFF0) == 0x0200) {
                lockedIdx = thisIdx;
                g_sport_debug.locked_poll_id = currentPollId;
                *currentAmps = (float)rawVal / 10.0f;
                g_sport_debug.current_amps = *currentAmps;
                g_sport_debug.successful_reads++;
                return true;
            }
            // AppID 0x0210..0x021F: Voltage (VFAS, 0.01 V units)
            else if ((appId & 0xFFF0) == 0x0210) {
                g_sport_debug.voltage_volts = (float)rawVal / 100.0f;
            }
        } else {
            g_sport_debug.last_crc_ok = false;
        }
    }

    /* Count misses only when locked; release the lock if the sensor goes silent */
    if (lockedIdx != SPORT_LOCK_NONE && !frameOk) {
        if (++lockMisses >= SPORT_LOCK_MAX_MISSES) {
            lockedIdx = SPORT_LOCK_NONE;
            lockMisses = 0;
            g_sport_debug.locked_poll_id = SPORT_LOCK_NONE;
        }
    }
    return false;
}


void SPort_GetDiagString(char *buf, uint16_t maxLen) {
    int whole_curr = (int)g_sport_debug.current_amps;
    int frac_curr = (int)((g_sport_debug.current_amps - (float)whole_curr) * 100.0f);
    if (frac_curr < 0) frac_curr = -frac_curr;

    int whole_volt = (int)g_sport_debug.voltage_volts;
    int frac_volt = (int)((g_sport_debug.voltage_volts - (float)whole_volt) * 10.0f);
    if (frac_volt < 0) frac_volt = -frac_volt;

    snprintf(buf, maxLen,
        "[SPort] Wire PA7: %s | Clock: %luMHz | Poll: 0x%02X | Rx: %d/8 bytes | CRC: %s | I: %d.%02d A\r\n",
        g_sport_debug.wire_state_idle ? "HIGH (Warning!)" : "LOW (OK)",
        g_sport_debug.clock_hz / 1000000UL,
        g_sport_debug.last_poll_id,
        g_sport_debug.bytes_received,
        g_sport_debug.last_crc_ok ? "OK" : "NONE/FAIL",
        whole_curr, frac_curr
    );
}


