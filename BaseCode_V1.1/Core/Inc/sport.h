/*
 * sport.h
 *
 *  Created on: Oct 26, 2026
 *      Author: daivi
 */

#ifndef INC_SPORT_H_
#define INC_SPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t clock_hz;
    uint32_t bit_cycles;
    uint8_t last_poll_id;
    uint8_t bytes_received;
    uint8_t raw_frame[8];
    uint16_t last_app_id;
    uint32_t last_raw_val;
    float current_amps;
    float voltage_volts;
    bool last_crc_ok;
    bool wire_state_idle;
    uint32_t successful_reads;
    uint32_t total_polls;
    uint8_t locked_poll_id;   /* Physical ID locked onto after a current reply (0xFF = scanning) */
} SPort_DebugInfo_t;

extern SPort_DebugInfo_t g_sport_debug;

/**
 * @brief Initialize SmartPort on PA7 (57600 baud, inverted logic, DWT cycle counter).
 */
void SPort_Init(void);

/**
 * @brief Send SmartPort poll header (0x7E + Physical ID).
 * @param pollId Physical ID with embedded CRC (e.g. 0x22, 0xE4).
 */
void SPort_SendPollHeader(uint8_t pollId);

/**
 * @brief Read a single byte from SmartPort (PA7) at 57600 baud, inverted logic.
 * @param out Pointer to store received byte
 * @param timeoutUs Timeout in microseconds
 * @return true if byte received successfully, false on timeout
 */
bool SPort_ReadByte(uint8_t *out, uint32_t timeoutUs);

/**
 * @brief Poll SmartPort for current sensor data (and other sensors).
 * @param currentAmps Pointer to store current in Amperes
 * @return true if a valid current reading was received, false otherwise
 */
bool SPort_PollCurrent(float *currentAmps);

/**
 * @brief Get human-readable diagnostic status string.
 * @param buf Output buffer
 * @param maxLen Buffer size
 */
void SPort_GetDiagString(char *buf, uint16_t maxLen);

#ifdef __cplusplus
}
#endif
#endif /* INC_SPORT_H_ */
