#ifndef TASKS_TASK_MANAGER_H
#define TASKS_TASK_MANAGER_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "algorithm.h"
#include "drivers/hw_encoders.h"
#include "drivers/motor_driver.h"
#include "drivers/bno085.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * OPTIONAL SENSOR FEATURE FLAG
 * Set to 0: Optical Flow & ToF completely removed from execution and queues (zero overhead).
 * Set to 1: Automatically enables Optical Flow & ToF reading in AcquisitionTask.
 */
#define ENABLE_OPTICAL_FLOW_TOF 0

#if (ENABLE_OPTICAL_FLOW_TOF == 1)
#include "drivers/optical_flow_tof.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Rate Monotonic Priorities (Higher number = Higher preemption priority) */
#define PRIORITY_MOTOR_TASK        ( tskIDLE_PRIORITY + 4 ) /* 100 Hz / 10 ms (Highest rate, safety critical) */
#define PRIORITY_ACQUISITION_TASK  ( tskIDLE_PRIORITY + 3 ) /* 50 Hz / 20 ms  (Sensors) */
#define PRIORITY_ESTIMATION_TASK   ( tskIDLE_PRIORITY + 2 ) /* 50 Hz / 20 ms  (Dead reckoning pose integration) */
#define PRIORITY_COMMUNICATION_TASK  ( tskIDLE_PRIORITY + 1 ) /* Event/Rate driven (USB Telemetry & Parsing) */

/* Queue Lengths */
#define Q_SENSOR_DATA_LEN   4
#define Q_ROBOT_STATE_LEN   4
#define Q_MOTOR_CMD_LEN     4
#define Q_USB_RX_LEN      256

/* Deadman Safety Watchdog Timeout: 150 ms threshold */
#define DEADMAN_TIMEOUT_MS  150

/* Sensor Data Message Container */
typedef struct {
    hw_encoder_data_t       encoders;
    bno085_reading_t        imu;
#if (ENABLE_OPTICAL_FLOW_TOF == 1)
    optical_flow_tof_data_t flow_tof;
#endif
    uint32_t                timestamp_tick;
} sensor_data_t;

/* Motor Command Message Container */
typedef struct {
    int16_t  speeds[NUM_MOTORS];
    uint32_t timestamp_tick;
} motor_cmd_t;

/* Estimated Robot State Container */
typedef struct {
    robot_pose_t pose;
    int16_t      motor_pwms[NUM_MOTORS];
    bool         bno_ready;
    uint32_t     timestamp_tick;
} robot_state_t;

/* Global Static Queues */
extern QueueHandle_t q_sensor_data;
extern QueueHandle_t q_robot_state;
extern QueueHandle_t q_motor_cmd;
extern QueueHandle_t q_usb_rx;

/* Global Synchronization Primitives */
extern SemaphoreHandle_t xUsbTxSemaphore;
extern SemaphoreHandle_t xRobotStateMutex;

/**
 * @brief Initialize all static memory queues, semaphores, mutexes, and tasks.
 */
void task_manager_init(void);

/**
 * @brief Record a fresh packet reception timestamp from Jetson (resets deadman timer).
 */
void task_manager_notify_jetson_packet_received(void);

/**
 * @brief Get last Jetson packet arrival tick for the deadman safety loop.
 */
uint32_t task_manager_get_last_jetson_packet_tick(void);

/**
 * @brief Update the protected global snapshot of the robot state.
 */
void task_manager_set_state_snapshot(const robot_state_t *new_state);

/**
 * @brief Safely copy the latest robot state snapshot under mutex lock.
 */
void task_manager_get_state_snapshot(robot_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_TASK_MANAGER_H */
