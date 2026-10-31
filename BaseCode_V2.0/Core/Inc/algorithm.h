#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pure Math Constants defined by specifications */
#define ROBOT_WHEEL_RADIUS_MM   35.0f   /* Wheel radius R = 35 mm */
#define ROBOT_WHEELBASE_MM     150.0f   /* Wheelbase W = 150 mm */
#define ENCODER_TICKS_PER_REV  2000.0f  /* CPR for dead-wheel encoders */

#define PI_FLOAT 3.14159265358979323846f

typedef struct {
    float x_mm;          /* Integrated X position in mm */
    float y_mm;          /* Integrated Y position in mm */
    float heading_rad;   /* Integrated heading in radians [-PI, PI] */
    float heading_deg;   /* Heading in degrees [-180, 180] */
    float linear_vel_m_s;/* Forward velocity in m/s */
    float angular_vel_rad_s; /* Rotational velocity in rad/s */
} robot_pose_t;

/**
 * @brief Initialize robot pose to origin (0, 0, 0).
 */
void algorithm_init(robot_pose_t *pose);

/**
 * @brief Dead reckoning integration using wheel radius R=35mm and wheelbase W=150mm.
 *
 * @param pose In/Out pose structure to update.
 * @param delta_left   Left encoder tick change over interval dt.
 * @param delta_right  Right encoder tick change over interval dt.
 * @param delta_aux    Auxiliary/perpendicular encoder tick change (if using 3 dead wheels).
 * @param dt_seconds   Time step in seconds (e.g. 0.02s for 50Hz).
 * @param imu_yaw_deg  External IMU yaw in degrees (-180..180) for heading fusion.
 * @param use_imu_yaw  If true, fuses/corrects heading with absolute IMU yaw.
 */
void algorithm_update_odometry(robot_pose_t *pose,
                               int16_t delta_left,
                               int16_t delta_right,
                               int16_t delta_aux,
                               float dt_seconds,
                               float imu_yaw_deg,
                               bool use_imu_yaw);

/**
 * @brief Normalize angle to [-PI, PI].
 */
float algorithm_normalize_angle_rad(float angle);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_H */
