#include "algorithm.h"
#include <math.h>

void algorithm_init(robot_pose_t *pose)
{
    if (!pose) return;
    pose->x_mm              = 0.0f;
    pose->y_mm              = 0.0f;
    pose->heading_rad       = 0.0f;
    pose->heading_deg       = 0.0f;
    pose->linear_vel_m_s    = 0.0f;
    pose->angular_vel_rad_s = 0.0f;
}

float algorithm_normalize_angle_rad(float angle)
{
    while (angle > PI_FLOAT)  angle -= (2.0f * PI_FLOAT);
    while (angle < -PI_FLOAT) angle += (2.0f * PI_FLOAT);
    return angle;
}

void algorithm_update_odometry(robot_pose_t *pose,
                               int16_t delta_left,
                               int16_t delta_right,
                               int16_t delta_aux,
                               float dt_seconds,
                               float imu_yaw_deg,
                               bool use_imu_yaw)
{
    if (!pose || dt_seconds <= 0.0f) return;

    /* Distance in mm traveled per encoder count: (2 * PI * R) / CPR */
    const float mm_per_count = (2.0f * PI_FLOAT * ROBOT_WHEEL_RADIUS_MM) / ENCODER_TICKS_PER_REV;

    float ds_left   = (float)delta_left * mm_per_count;
    float ds_right  = (float)delta_right * mm_per_count;
    float ds_strafe = (float)delta_aux * mm_per_count;

    /* Forward linear displacement in robot frame */
    float ds_forward = (ds_left + ds_right) * 0.5f;

    /* Heading delta calculated from wheel differential: (ds_r - ds_l) / W */
    float d_theta_rad = (ds_right - ds_left) / ROBOT_WHEELBASE_MM;

    /* Calculate midpoint heading for arc approximation */
    float theta_mid = pose->heading_rad + (d_theta_rad * 0.5f);

    /* Integrate position to global coordinate frame */
    pose->x_mm += (ds_forward * cosf(theta_mid)) - (ds_strafe * sinf(theta_mid));
    pose->y_mm += (ds_forward * sinf(theta_mid)) + (ds_strafe * cosf(theta_mid));

    /* Update heading */
    if (use_imu_yaw) {
        /* Direct fusion with drift-compensated IMU heading */
        float imu_rad = imu_yaw_deg * (PI_FLOAT / 180.0f);
        pose->heading_rad = algorithm_normalize_angle_rad(imu_rad);
    } else {
        pose->heading_rad = algorithm_normalize_angle_rad(pose->heading_rad + d_theta_rad);
    }
    pose->heading_deg = pose->heading_rad * (180.0f / PI_FLOAT);

    /* Instantaneous velocities */
    pose->linear_vel_m_s    = (ds_forward / 1000.0f) / dt_seconds;
    pose->angular_vel_rad_s = d_theta_rad / dt_seconds;
}
