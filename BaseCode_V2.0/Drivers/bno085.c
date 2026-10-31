#include "bno085.h"
#include "state_machines/imu_sm.h"
#include "bno085_driver.h"
#include "bno_port.h"

BNO085   g_bno;
BNO_Port g_bno_port;

static bno085_reading_t s_last_reading = {0};

void bno085_hw_init(void)
{
    imu_sm_init();
}

void bno085_hw_step(void)
{
    /* Step state machine */
    if (!imu_sm_is_ready()) {
        imu_sm_dispatch(IMU_EVT_STEP);
    } else {
        /* Run normal update */
        imu_sm_dispatch(IMU_EVT_STEP);

        /* Update coherent reading cache */
        BNO_Quaternion q;
        float yaw = 0.0f;
        bool has_rot = (BNO085_GetGameRotation(&g_bno, &q) || BNO085_GetRotation(&g_bno, &q));
        if (has_rot) {
            yaw = BNO085_GetEulerYaw(&q);
        }

        BNO_Vector3 accel = {0.0f, 0.0f, 0.0f};
        bool has_accel = (BNO085_GetLinearAccel(&g_bno, &accel) || BNO085_GetAccel(&g_bno, &accel));

        s_last_reading.yaw     = yaw;
        s_last_reading.pitch   = 0.0f;
        s_last_reading.roll    = 0.0f;
        s_last_reading.accel_x = accel.x;
        s_last_reading.accel_y = accel.y;
        s_last_reading.accel_z = accel.z;
        s_last_reading.valid   = (has_rot || has_accel);
    }
}

void bno085_hw_get_reading(bno085_reading_t *out_reading)
{
    if (!out_reading) return;
    *out_reading = s_last_reading;
}

bool bno085_hw_is_ready(void)
{
    return imu_sm_is_ready();
}
