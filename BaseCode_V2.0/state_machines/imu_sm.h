#ifndef STATE_MACHINES_IMU_SM_H
#define STATE_MACHINES_IMU_SM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_STATE_UNINIT = 0,
    IMU_STATE_DETECT,
    IMU_STATE_INIT_DRIVER,
    IMU_STATE_CONFIG_REPORTS,
    IMU_STATE_RUNNING,
    IMU_STATE_BUS_RECOVERY,
    IMU_STATE_ERROR,
    IMU_STATE_COUNT
} imu_state_t;

typedef enum {
    IMU_EVT_STEP = 0,
    IMU_EVT_SUCCESS,
    IMU_EVT_FAIL,
    IMU_EVT_BUS_ERROR,
    IMU_EVT_COUNT
} imu_event_t;

typedef void (*imu_action_fn_t)(void);

typedef struct {
    imu_state_t       next_state;
    imu_action_fn_t   action;
} imu_transition_t;

/**
 * @brief Initialize the IMU 2D State Matrix.
 */
void imu_sm_init(void);

/**
 * @brief Dispatch an event to the 2D State Matrix transition table.
 */
void imu_sm_dispatch(imu_event_t evt);

/**
 * @brief Get current IMU state machine state.
 */
imu_state_t imu_sm_get_state(void);

/**
 * @brief Returns true if IMU is fully operational and streaming valid data.
 */
bool imu_sm_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* STATE_MACHINES_IMU_SM_H */
