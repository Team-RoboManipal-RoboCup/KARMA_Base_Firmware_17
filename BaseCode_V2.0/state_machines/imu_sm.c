#include "imu_sm.h"
#include "drivers/i2c_recovery.h"
#include "bno085_driver.h"
#include "bno_port.h"
#include "main.h"

/* Forward declare state machine functions & action functions */
void imu_sm_dispatch(imu_event_t evt);

static void action_none(void);
static void action_do_detect(void);
static void action_do_init_driver(void);
static void action_do_config_reports(void);
static void action_do_run(void);
static void action_do_bus_recovery(void);
static void action_do_error(void);

static imu_state_t s_current_state = IMU_STATE_UNINIT;

/*
 * 2D State Matrix (Transition Table):
 * Index 1: Current State [IMU_STATE_COUNT]
 * Index 2: Triggering Event [IMU_EVT_COUNT]
 * Value: { Next State, Action Function Pointer }
 */
static const imu_transition_t s_transition_table[IMU_STATE_COUNT][IMU_EVT_COUNT] = {
    /* Current State: IMU_STATE_UNINIT */
    [IMU_STATE_UNINIT] = {
        [IMU_EVT_STEP]      = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    /* Current State: IMU_STATE_DETECT */
    [IMU_STATE_DETECT] = {
        [IMU_EVT_STEP]      = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_INIT_DRIVER,   action_do_init_driver },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    /* Current State: IMU_STATE_INIT_DRIVER */
    [IMU_STATE_INIT_DRIVER] = {
        [IMU_EVT_STEP]      = { IMU_STATE_INIT_DRIVER,   action_do_init_driver },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_CONFIG_REPORTS,action_do_config_reports },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    /* Current State: IMU_STATE_CONFIG_REPORTS */
    [IMU_STATE_CONFIG_REPORTS] = {
        [IMU_EVT_STEP]      = { IMU_STATE_CONFIG_REPORTS,action_do_config_reports },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_RUNNING,       action_do_run },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    /* Current State: IMU_STATE_RUNNING */
    [IMU_STATE_RUNNING] = {
        [IMU_EVT_STEP]      = { IMU_STATE_RUNNING,       action_do_run },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_RUNNING,       action_none },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    /* Current State: IMU_STATE_BUS_RECOVERY */
    [IMU_STATE_BUS_RECOVERY] = {
        [IMU_EVT_STEP]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_FAIL]      = { IMU_STATE_ERROR,         action_do_error },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_ERROR,         action_do_error }
    },
    /* Current State: IMU_STATE_ERROR */
    [IMU_STATE_ERROR] = {
        [IMU_EVT_STEP]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_FAIL]      = { IMU_STATE_ERROR,         action_none },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_ERROR,         action_none }
    }
};

/* Driver objects referenced by actions */
extern BNO085   g_bno;
extern BNO_Port g_bno_port;

static void action_none(void)
{
    /* No-op */
}

static void action_do_detect(void)
{
    BNO_Port_Init(&g_bno_port);
    if (BNO_Port_Detect()) {
        imu_sm_dispatch(IMU_EVT_SUCCESS);
    } else {
        imu_sm_dispatch(IMU_EVT_FAIL);
    }
}

static void action_do_init_driver(void)
{
    if (BNO085_Init(&g_bno, &g_bno_port) == BNO_PORT_OK) {
        /* Drain initial startup/advertisement packets */
        for (int i = 0; i < 10; i++) {
            BNO085_Update(&g_bno);
        }
        imu_sm_dispatch(IMU_EVT_SUCCESS);
    } else {
        imu_sm_dispatch(IMU_EVT_FAIL);
    }
}

static void action_do_config_reports(void)
{
    /* 50 Hz = 20,000 us */
    BNO085_EnableGameRotation(&g_bno, 20000);
    BNO085_EnableRotation(&g_bno, 20000);
    BNO085_EnableLinearAccel(&g_bno, 20000);
    BNO085_EnableAccel(&g_bno, 20000);

    imu_sm_dispatch(IMU_EVT_SUCCESS);
}

static void action_do_run(void)
{
    int errors = 0;
    for (int i = 0; i < 4; i++) {
        int8_t res = BNO085_Update(&g_bno);
        if (res != BNO_PORT_OK) {
            errors++;
        }
    }
    if (errors >= 4) {
        /* Bus stalled or sensor not responding */
        imu_sm_dispatch(IMU_EVT_BUS_ERROR);
    }
}

static void action_do_bus_recovery(void)
{
    /* Perform 9-clock I2C bus clear */
    if (i2c_bus_recovery_perform()) {
        imu_sm_dispatch(IMU_EVT_SUCCESS);
    } else {
        imu_sm_dispatch(IMU_EVT_FAIL);
    }
}

static void action_do_error(void)
{
    /* In error state, line was held low even after recovery */
}

void imu_sm_init(void)
{
    s_current_state = IMU_STATE_UNINIT;
    imu_sm_dispatch(IMU_EVT_STEP);
}

void imu_sm_dispatch(imu_event_t evt)
{
    if (evt >= IMU_EVT_COUNT || s_current_state >= IMU_STATE_COUNT) {
        return;
    }

    imu_transition_t transition = s_transition_table[s_current_state][evt];
    if (transition.next_state < IMU_STATE_COUNT) {
        s_current_state = transition.next_state;
    }

    if (transition.action != NULL && transition.action != action_none) {
        transition.action();
    }
}

imu_state_t imu_sm_get_state(void)
{
    return s_current_state;
}

bool imu_sm_is_ready(void)
{
    return (s_current_state == IMU_STATE_RUNNING);
}
