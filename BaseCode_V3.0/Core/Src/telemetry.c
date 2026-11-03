/*
 * telemetry.c
 * Robot telemetry JSON formatter.
 */

#include "telemetry.h"

#include <math.h>
#include <stdio.h>

/*
 * Format floating-point values to 2 decimal places.
 * Non-finite values are represented as JSON null.
 */
static void format_float_2(
    char *buffer,
    size_t buffer_size,
    float value)
{
    if (buffer == NULL || buffer_size == 0U)
        return;

    if (isfinite((double)value))
    {
        (void)snprintf(buffer, buffer_size, "%.2f", (double)value);
    }
    else
    {
        (void)snprintf(buffer, buffer_size, "null");
    }
}


bool Telemetry_FormatJson(
    const RobotTelemetry *data,
    char *out_buffer,
    size_t out_capacity,
    size_t *out_length)
{
    int written;

    /* Validate arguments */
    if (out_length != NULL)
        *out_length = 0U;

    if (data == NULL ||
        out_buffer == NULL ||
        out_length == NULL ||
        out_capacity == 0U)
    {
        return false;
    }

    out_buffer[0] = '\0';

    /* Format floating-point values */
    char yaw[24];
    char ax[24];
    char ay[24];
    char wx[24];
    char wy[24];
    char cur[24];
    char vb[24];

    format_float_2(yaw, sizeof(yaw), data->yaw);
    format_float_2(ax, sizeof(ax), data->ax);
    format_float_2(ay, sizeof(ay), data->ay);
    format_float_2(wx, sizeof(wx), data->wx);
    format_float_2(wy, sizeof(wy), data->wy);
    format_float_2(cur, sizeof(cur), data->cur);
    format_float_2(vb, sizeof(vb), data->vb);

    /*
     * Original main(3).c JSON fields and ordering preserved.
     * ToF and optical-flow fields appended.
     */

    written = snprintf(
        out_buffer,
        out_capacity,

        "{\"t\":%lu,\"yaw\":%s,\"ax\":%s,\"ay\":%s,"
        "\"wx\":%s,\"wy\":%s,"
        "\"e1\":%ld,\"e2\":%ld,\"e3\":%ld,\"ok\":%u,"
        "\"pwm\":[%d,%d,%d,%d],"
        "\"cur\":%s,\"vb\":%s,\"cs\":%u,"
        "\"tof\":%lu,\"tof_ok\":%u,"
        "\"ofx\":%d,\"ofy\":%d,\"ofq\":%u,\"of_ok\":%u}\r\n",

        (unsigned long)data->t,

        yaw,
        ax,
        ay,
        wx,
        wy,

        (long)data->e1,
        (long)data->e2,
        (long)data->e3,

        (unsigned int)(data->ok != 0U),

        (int)data->pwm[0],
        (int)data->pwm[1],
        (int)data->pwm[2],
        (int)data->pwm[3],

        cur,
        vb,

        (unsigned int)(data->cs != 0U),

        (unsigned long)data->tof,
        (unsigned int)(data->tof_ok != 0U),

        (int)data->ofx,
        (int)data->ofy,
        (unsigned int)data->ofq,
        (unsigned int)(data->of_ok != 0U)
    );

    /* Check for formatting errors or insufficient buffer capacity */
    if (written < 0 || (size_t)written >= out_capacity)
    {
        out_buffer[0] = '\0';
        return false;
    }

    *out_length = (size_t)written;

    return true;
}
