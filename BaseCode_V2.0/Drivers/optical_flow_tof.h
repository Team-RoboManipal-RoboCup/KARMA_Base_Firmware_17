#ifndef DRIVERS_OPTICAL_FLOW_TOF_H
#define DRIVERS_OPTICAL_FLOW_TOF_H

/*
 * OPTIONAL SENSOR DRIVER TEMPLATE
 * Currently inactive (ENABLE_OPTICAL_FLOW_TOF = 0 in tasks/task_manager.h).
 * To enable in the future:
 *   1. Set #define ENABLE_OPTICAL_FLOW_TOF 1 in tasks/task_manager.h
 *   2. Connect sensor hardware (SPI/I2C) and implement reading logic in optical_flow_tof.c
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    flow_x;       /* Optical flow X velocity/displacement */
    float    flow_y;       /* Optical flow Y velocity/displacement */
    float    tof_distance; /* Distance to surface (Z elevation in meters) */
    uint8_t  flow_quality; /* Surface tracking confidence 0-255 */
    bool     valid;        /* Sensor data validity flag */
} optical_flow_tof_data_t;

void optical_flow_tof_init(void);
void optical_flow_tof_read(optical_flow_tof_data_t *out_data);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_OPTICAL_FLOW_TOF_H */
