/*
 * app_config.h
 * Application-wide configuration for BaseCode_V3.0
 *
 * Keep hardware pin assignments and peripheral configuration in main.h
 * and the CubeMX .ioc file. This file contains application-level settings.
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

/* -------------------------------------------------------------------------- */
/* Firmware                                                                    */
/* -------------------------------------------------------------------------- */

#define FIRMWARE_VERSION                    "1.1"

/* -------------------------------------------------------------------------- */
/* Robot hardware topology                                                     */
/* -------------------------------------------------------------------------- */

#define MOTOR_COUNT                         4U
#define ENCODER_COUNT                       3U

/* Motor command is expressed on a -1000 to +1000 scale. The driver limits it
 * to +/-500, which corresponds to a maximum 50% PWM duty command. */
#define MOTOR_MAX_PWM_LIMIT                 500

/* Target PWM frequency. The actual TIM5 prescaler/ARR must be configured in
 * CubeMX / MX_TIM5_Init() to match the active TIM5 clock. For the current
 * BaseCode_V3.0 clock configuration (TIM5 clock = 16 MHz): PSC = 0, ARR = 1999. */
#define MOTOR_PWM_FREQUENCY_HZ               8000U

/* -------------------------------------------------------------------------- */
/* Sensor enable switches                                                      */
/* Set a switch to 0U to compile out/skip that subsystem in application code. */
/* -------------------------------------------------------------------------- */

#define APP_ENABLE_BNO085                    1U
#define APP_ENABLE_TOF                       1U
#define APP_ENABLE_OPTICAL_FLOW              1U
#define APP_ENABLE_FRSKY_CURRENT             1U

/* -------------------------------------------------------------------------- */
/* BNO085                                                                      */
/* -------------------------------------------------------------------------- */

/* 20,000 us = 50 Hz report interval, matching the existing main(3).c setup. */
#define BNO085_REPORT_INTERVAL_US             20000U
#define BNO085_MAX_PACKETS_PER_LOOP           8U
#define BNO085_FEATURE_RETRY_MS               250U

/* Preserve the report selections used by the existing main(3).c. */
#define BNO085_ENABLE_GAME_ROTATION            1U
#define BNO085_ENABLE_ROTATION_FALLBACK       1U
#define BNO085_ENABLE_LINEAR_ACCEL            1U
#define BNO085_ENABLE_ACCEL_FALLBACK          1U
#define BNO085_ENABLE_GYRO                    1U

/* -------------------------------------------------------------------------- */
/* ToF                                                                        */
/* -------------------------------------------------------------------------- */

/* Provisional starting values: confirm against the exact ToF model and its
 * measurement timing budget before finalizing. 50 ms = 20 Hz. */
#define TOF_UPDATE_INTERVAL_MS                50U
#define TOF_STALE_TIMEOUT_MS                  250U

/* -------------------------------------------------------------------------- */
/* Optical flow                                                               */
/* -------------------------------------------------------------------------- */

/* Provisional starting values: confirm against the exact optical-flow sensor
 * model, frame rate and SPI transaction timing. 10 ms = 100 Hz. */
#define OPTICAL_FLOW_UPDATE_INTERVAL_MS       10U
#define OPTICAL_FLOW_STALE_TIMEOUT_MS         100U

/* -------------------------------------------------------------------------- */
/* Encoders                                                                    */
/* -------------------------------------------------------------------------- */

/* Preserve the existing 50 Hz encoder update cadence used by main(3).c,
 * where encoder_update() is called during each 20 ms telemetry cycle. */
#define ENCODER_UPDATE_INTERVAL_MS            20U

/* -------------------------------------------------------------------------- */
/* FrSky SmartPort current sensor                                              */
/* -------------------------------------------------------------------------- */

/* These values preserve the polling and freshness settings in the earlier
 * main.h. SmartPort is bit-banged and its polling routine is blocking. */
#define SPORT_POLL_OFFSET_MS                  8U
#define SPORT_POLL_PERIOD_MS                  12U
#define SPORT_STALE_MS                        1000U

/* -------------------------------------------------------------------------- */
/* USB CDC and telemetry                                                       */
/* -------------------------------------------------------------------------- */

/* Preserve the existing JSON telemetry rate and line-oriented protocol.
 * The telemetry packet format/keys remain defined by telemetry.c. */
#define TELEMETRY_INTERVAL_MS                 20U  /* 50 Hz */

/* Buffer capacities matching the existing main(3).c allocation. */
#define USB_TX_BUFFER_SIZE                    384U
#define USB_ACK_BUFFER_SIZE                   256U
#define USB_RX_LINE_SIZE                      128U

/* -------------------------------------------------------------------------- */
/* Motor command safety                                                        */
/* -------------------------------------------------------------------------- */

/* Stop motors if no command has been received within this interval.
 * Set to 0U to disable the watchdog, provided robot_loop() honors this macro. */
#define MOTOR_WATCHDOG_TIMEOUT                1500U

#endif /* INC_APP_CONFIG_H_ */
