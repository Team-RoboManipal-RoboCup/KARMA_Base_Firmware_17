/*
 * telemetry.h
 * Robot telemetry data contract and JSON formatter interface.
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Robot telemetry snapshot
 *
 * Original main(3).c fields are preserved.
 * New ToF and optical-flow fields are appended.
 */

typedef struct
{
    /* Original telemetry fields */

    uint32_t t;

    float yaw;
    float ax;
    float ay;
    float wx;
    float wy;

    int32_t e1;
    int32_t e2;
    int32_t e3;

    uint8_t ok;

    int16_t pwm[MOTOR_COUNT];

    float cur;
    float vb;
    uint8_t cs;


    /* Additional sensor fields */

    uint32_t tof;
    uint8_t tof_ok;

    int16_t ofx;
    int16_t ofy;
    uint16_t ofq;
    uint8_t of_ok;

} RobotTelemetry;


/*
 * Format one compact JSON line.
 *
 * Output ends with \r\n and is NUL terminated.
 *
 * Returns true if the complete JSON fits in the buffer.
 * out_length excludes the terminating NUL.
 */

bool Telemetry_FormatJson(
    const RobotTelemetry *data,
    char *out_buffer,
    size_t out_capacity,
    size_t *out_length
);

#ifdef __cplusplus
}
#endif

#endif /* INC_TELEMETRY_H_ */
