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

#include <DRIVERS/sport.h>
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
 * STM32F411 PB12 GPIO Register Mapping:
 * - MODER bits [25:24]: 00 = Input, 01 = Output
 * - PUPDR bits [25:24]: 00 = No pull, 01 = Pull-Up, 10 = Pull-Down
 * - OSPEEDR bits [25:24]: 11 = Very High Speed
 * - OTYPER bit 12: 0 = Push-Pull
 * - BSRR bit 12 = Set HIGH, bit 28 = Reset LOW
 * - IDR bit 12 = Read Pin
 */

#define PB12_SET_OUTPUT() do { \
    GPIOB->MODER = (GPIOB->MODER & ~(0x3U << 24)) | (0x1U << 24); \
    GPIOB->PUPDR &= ~(0x3U << 24); \
    GPIOB->OTYPER &= ~(1U << 12); \
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0x3U << 24)) | (0x3U << 24); \
} while(0)

#define PB12_SET_INPUT() do { \
    GPIOB->MODER &= ~(0x3U << 24); \
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(0x3U << 24)) | (0x2U << 24); \
    GPIOB->BSRR = ((uint32_t)GPIO_PIN_12 << 16U); \
} while(0)


#define PB12_HIGH() (GPIOB->BSRR = GPIO_PIN_12)
#define PB12_LOW() (GPIOB->BSRR = ((uint32_t)GPIO_PIN_12 << 16U))
#define PB12_READ() ((GPIOB->IDR & GPIO_PIN_12) != 0U)

static uint32_t bitCycles;
//static bool s_rx_frame_active = false;

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



void SPort_Init(void)
{
    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Configure PB12: Very High Speed, Push-Pull */
    GPIOB->OSPEEDR |= (0x3U << 24U);
    GPIOB->OTYPER  &= ~(1U << 12U);
    /* Enable DWT Cycle Counter on Cortex-M4 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Unlock CoreSight */
    *(volatile uint32_t *)0xE0001FB0U = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* Calculate bit timing for 57600 baud */
    SystemCoreClockUpdate();
    bitCycles = SystemCoreClock / 57600UL;
    g_sport_debug.clock_hz = SystemCoreClock;
    g_sport_debug.bit_cycles = bitCycles;
    g_sport_debug.locked_poll_id = SPORT_LOCK_NONE;
    /* Start in input mode with pull-down */
    PB12_SET_INPUT();
}

void SPort_SendPollHeader(uint8_t pollId) {
	uint32_t irq_state = __get_PRIMASK();

    PB12_SET_OUTPUT();
    __disable_irq();

    // 1. Transmit 0x7E
    uint32_t start = DWT->CYCCNT;
    uint32_t target = bitCycles;
    PB12_HIGH(); // Inverted Start Bit (HIGH)
    waitUntilCycle(start, target);

    uint8_t b = 0x7E;
    for (uint8_t i = 0; i < 8; i++) {
        target += bitCycles;
        if (b & 1) PB12_LOW();
        else       PB12_HIGH();
        b >>= 1;
        waitUntilCycle(start, target);
    }
    target += bitCycles;
    PB12_LOW(); // Inverted Stop Bit (LOW)
    waitUntilCycle(start, target);

    // 2. Transmit Physical ID byte immediately
    start = DWT->CYCCNT;
    target = bitCycles;
    PB12_HIGH(); // Inverted Start Bit (HIGH)
    waitUntilCycle(start, target);

    b = pollId;
    for (uint8_t i = 0; i < 8; i++) {
        target += bitCycles;
        if (b & 1) PB12_LOW();
        else       PB12_HIGH();
        b >>= 1;
        waitUntilCycle(start, target);
    }
    target += bitCycles;
    PB12_LOW(); // Inverted Stop Bit (LOW)
    waitUntilCycle(start, target);

    /* Release the SmartPort bus before servicing pending interrupts. */
    PB12_SET_INPUT();

    if (irq_state == 0U) {
        __enable_irq();
    }
}

bool SPort_ReadByte(uint8_t *out, uint32_t timeoutUs)
{
    if ((out == NULL) || (bitCycles == 0U))
    {
        return false;
    }

    uint32_t irq_state = __get_PRIMASK();
    uint32_t timeoutCycles =
        (uint32_t)(((uint64_t)timeoutUs * SystemCoreClock) / 1000000ULL);
    uint32_t waitStart = DWT->CYCCNT;
    /* Wait for inverted UART start bit: wire goes HIGH */
    while (!PB12_READ())
    {
        if ((DWT->CYCCNT - waitStart) >= timeoutCycles)
        {
            return false;
        }
    }

    /* Mask interrupts only while sampling this byte */
    __disable_irq();

    uint32_t start = DWT->CYCCNT;
    uint32_t target = bitCycles + (bitCycles / 2U);

    /* Sample at the centre of data bit 0 */
    waitUntilCycle(start, target);
    uint8_t val = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (!PB12_READ())
        {
            val |= (uint8_t)(1U << i);
        }

        target += bitCycles;
        waitUntilCycle(start, target);
    }

    /* Move to the centre of the stop bit */
    //target += bitCycles;
    //waitUntilCycle(start, target);

    /* Inverted stop bit should be LOW */
    bool stopOk = !PB12_READ();
    *out = val;
    /* Restore the interrupt state that existed on entry */
    if (irq_state == 0U)
    {
        __enable_irq();
    }
    return stopOk;
}

bool SPort_PollCurrent(float *currentAmps)
{
    if ((currentAmps == NULL) || (bitCycles == 0U))
    {
        return false;
    }

    uint8_t thisIdx;

    /* Continue polling the locked sensor if one was identified */
    if (lockedIdx != SPORT_LOCK_NONE)
    {
        thisIdx = lockedIdx;
    }
    else
    {
        thisIdx = pollIdx;

        pollIdx = (uint8_t)(
            (pollIdx + 1U) %
            (sizeof(pollIds) / sizeof(pollIds[0]))
        );
    }

    uint8_t currentPollId = pollIds[thisIdx];

    g_sport_debug.last_poll_id = currentPollId;
    g_sport_debug.wire_state_idle = PB12_READ();
    g_sport_debug.total_polls++;
    g_sport_debug.last_crc_ok = false;

    SPort_SendPollHeader(currentPollId);

    uint8_t frame[8] = {0U};
    uint8_t count = 0U;
    bool stuffed = false;
    bool frameOk = false;

    /*
     * Receive the eight-byte SmartPort response.
     * Interrupts are managed by SPort_ReadByte().
     */
    while (count < 8U)
    {
        uint8_t b;
        uint32_t timeout = (count == 0U) ? 5000U : 2000U;

        if (!SPort_ReadByte(&b, timeout))
        {
            break;
        }

        if (b == 0x7D && !stuffed)
        {
            stuffed = true;
            continue;
        }

        if (stuffed)
        {
            b ^= 0x20U;
            stuffed = false;
        }

        frame[count++] = b;
    }

    /* An incomplete escape sequence makes the frame invalid */
    if (stuffed)
    {
        count = 0U;
    }

    g_sport_debug.bytes_received = count;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        g_sport_debug.raw_frame[i] = frame[i];
    }

    /* Validate frame type and length */
    if ((count == 8U) && (frame[0] == 0x10U))
    {
        uint16_t sum = 0U;

        for (uint8_t i = 0U; i < 8U; i++)
        {
            sum += frame[i];
            sum += (sum >> 8U);
            sum &= 0x00FFU;
        }

        if (sum == 0xFFU)
        {
            g_sport_debug.last_crc_ok = true;
            frameOk = true;
            lockMisses = 0U;

            uint16_t appId =
                (uint16_t)frame[1] |
                ((uint16_t)frame[2] << 8U);

            uint32_t rawVal =
                (uint32_t)frame[3] |
                ((uint32_t)frame[4] << 8U) |
                ((uint32_t)frame[5] << 16U) |
                ((uint32_t)frame[6] << 24U);

            g_sport_debug.last_app_id = appId;
            g_sport_debug.last_raw_val = rawVal;

            /* Current: AppID 0x0200–0x020F, units of 0.1 A */
            if ((appId & 0xFFF0U) == 0x0200U)
            {
                lockedIdx = thisIdx;
                g_sport_debug.locked_poll_id = currentPollId;

                *currentAmps = (float)rawVal / 10.0f;

                g_sport_debug.current_amps = *currentAmps;
                g_sport_debug.successful_reads++;

                return true;
            }

            /* Voltage: AppID 0x0210–0x021F, units of 0.01 V */
            else if ((appId & 0xFFF0U) == 0x0210U)
            {
                g_sport_debug.voltage_volts =
                    (float)rawVal / 100.0f;
            }
        }
    }

    /* Release sensor lock after consecutive unsuccessful polls */
    if ((lockedIdx != SPORT_LOCK_NONE) && !frameOk)
    {
        lockMisses++;

        if (lockMisses >= SPORT_LOCK_MAX_MISSES)
        {
            lockedIdx = SPORT_LOCK_NONE;
            lockMisses = 0U;
            g_sport_debug.locked_poll_id = SPORT_LOCK_NONE;
        }
    }

    return false;
}


void SPort_GetDiagString(char *buf, uint16_t maxLen)
{
    if ((buf == NULL) || (maxLen == 0U))
    {
        return;
    }

    int whole_curr = (int)g_sport_debug.current_amps;
    int frac_curr = (int)(
        (g_sport_debug.current_amps - (float)whole_curr) * 100.0f
    );

    if (frac_curr < 0)
        frac_curr = -frac_curr;

    int whole_volt = (int)g_sport_debug.voltage_volts;
    int frac_volt = (int)(
        (g_sport_debug.voltage_volts - (float)whole_volt) * 10.0f
    );

    if (frac_volt < 0)
        frac_volt = -frac_volt;

    snprintf(buf, (size_t)maxLen,
        "[SPort] Wire PB12: %s | Clock: %luMHz | Poll: 0x%02X | "
        "Rx: %d/8 bytes | CRC: %s | I: %d.%02d A | V: %d.%01d V\r\n",

        g_sport_debug.wire_state_idle ? "HIGH (Warning!)" : "LOW (OK)",
        (unsigned long)(g_sport_debug.clock_hz / 1000000UL),
        (unsigned int)g_sport_debug.last_poll_id,
        (int)g_sport_debug.bytes_received,
        g_sport_debug.last_crc_ok ? "OK" : "NONE/FAIL",
        whole_curr,
        frac_curr,
        whole_volt,
        frac_volt
    );
}


