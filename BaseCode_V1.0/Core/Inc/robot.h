/*
 * robot.h
 *
 * Header for Autonomous 4-Wheel Robot Base
 * Manages 3-Encoder Dead Wheel Odometry, BNO085 IMU, 4x MD30C Motor Drivers,
 * and USB CDC JSON Telemetry/Command Interface.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_INTERVAL_MS   20    /* 50 Hz JSON telemetry rate */
#define MOTOR_WATCHDOG_TIMEOUT  1500  /* Stop motors if no command within 1500 ms (0 to disable) */

/* Core Robot Functions */
void robot_init(void);
void robot_loop(void);
void robot_process_rx(uint8_t *buf, uint32_t len);

/* Direct Control Helpers */
void robot_set_motor(uint8_t motor_id, int16_t speed);
void robot_set_all_motors(int16_t speed);
void robot_stop(void);

#endif /* ROBOT_H */
