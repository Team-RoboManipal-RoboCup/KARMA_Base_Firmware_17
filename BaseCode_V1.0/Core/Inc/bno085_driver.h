#ifndef BNO_DRIVER_H
#define BNO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "bno_port.h"


/* ============================================================
 * Configuration
 * ============================================================ */

#define BNO085_RX_BUFFER_SIZE     256


/* ============================================================
 * SHTP channels
 * ============================================================ */

#define BNO_CHANNEL_COMMAND       0
#define BNO_CHANNEL_EXECUTABLE    1
#define BNO_CHANNEL_CONTROL       2
#define BNO_CHANNEL_INPUT         3
#define BNO_CHANNEL_WAKE_INPUT    4
#define BNO_CHANNEL_GYRO_ROTATION 5


/* ============================================================
 * SH-2 sensor report IDs
 * ============================================================ */

#define BNO_REPORT_ACCEL          0x01
#define BNO_REPORT_GYRO           0x02
#define BNO_REPORT_MAG            0x03
#define BNO_REPORT_LINEAR_ACCEL   0x04
#define BNO_REPORT_ROTATION       0x05
#define BNO_REPORT_GRAVITY        0x06
#define BNO_REPORT_GAME_ROTATION  0x08


/* ============================================================
 * SH-2 commands
 * ============================================================ */

#define BNO_CMD_SET_FEATURE       0xFD



/* ============================================================
 * Result
 * ============================================================ */

typedef enum
{
    BNO085_OK = 0,

    BNO085_ERROR,

    BNO085_ERROR_NOT_INITIALIZED,

    BNO085_ERROR_INVALID_ARGUMENT,

    BNO085_ERROR_BUFFER_OVERFLOW,

    BNO085_ERROR_I2C

} BNO085_Result;


/* ============================================================
 * Vector
 * ============================================================ */

typedef struct
{
    float x;
    float y;
    float z;

} BNO_Vector3;


/* ============================================================
 * Quaternion
 * ============================================================ */

typedef struct
{
    float x;
    float y;
    float z;
    float w;

} BNO_Quaternion;


/* ============================================================
 * Latest sensor values
 * ============================================================ */

typedef struct
{
    BNO_Vector3 accel;
    BNO_Vector3 gyro;
    BNO_Vector3 mag;

    BNO_Vector3 linear_accel;
    BNO_Vector3 gravity;

    BNO_Quaternion rotation;
    BNO_Quaternion game_rotation;

    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
    bool linear_accel_valid;
    bool gravity_valid;
    bool rotation_valid;
    bool game_rotation_valid;

} BNO085_Data;

typedef struct
{
    uint32_t rx_packets;      /* Total valid SHTP packets received */
    uint32_t rx_reports;      /* Total sensor reports parsed */
    uint32_t tx_packets;      /* Total SHTP packets transmitted */
    uint32_t tx_errors;       /* I2C transmit failures */
    uint32_t rx_errors;       /* I2C receive failures */
    uint16_t last_packet_len; /* Length of last received packet */
    uint8_t  last_channel;    /* Channel of last received packet */
    uint8_t  last_header[4];  /* Last 4-byte header received */
    uint8_t  i2c_addr;        /* Active I2C address (0x4A or 0x4B) */
} BNO085_Diag;


/* ============================================================
 * Driver object
 * ============================================================ */

typedef struct
{
    BNO_Port port;

    BNO085_Data data;
    BNO085_Diag diag;

    uint8_t rx_buffer[BNO085_RX_BUFFER_SIZE];
    uint8_t tx_sequence[6];
    uint8_t rx_sequence[6];

    volatile bool data_ready;

    bool initialized;

} BNO085;


/* ============================================================
 * Initialization & Reset
 * ============================================================ */

BNO_PortStatus BNO085_Init(
    BNO085 *dev,
    const BNO_Port *port
);

BNO_PortStatus BNO085_SoftReset(
    BNO085 *dev
);


/* ============================================================
 * Interrupt notification
 *
 * CALL THIS FROM YOUR EXTI CALLBACK (if INT pin is used).
 *
 * It does NOT perform I2C.
 * ============================================================ */

void BNO085_DataReady(
    BNO085 *dev
);


/* ============================================================
 * Process incoming data.
 *
 * Call this from your main loop/task.
 * ============================================================ */

BNO_PortStatus BNO085_Update(
    BNO085 *dev
);


/* ============================================================
 * Enable reports
 *
 * interval_us:
 *
 * 10000 = 100 Hz
 * 5000  = 200 Hz
 * 2000  = 500 Hz
 * ============================================================ */

BNO_PortStatus BNO085_EnableAccel(
    BNO085 *dev,
    uint32_t interval_us
);


BNO_PortStatus BNO085_EnableGyro(
    BNO085 *dev,
    uint32_t interval_us
);


BNO_PortStatus BNO085_EnableMag(
    BNO085 *dev,
    uint32_t interval_us
);


BNO_PortStatus BNO085_EnableLinearAccel(
    BNO085 *dev,
    uint32_t interval_us
);


BNO_PortStatus BNO085_EnableGravity(
    BNO085 *dev,
    uint32_t interval_us
);


BNO_PortStatus BNO085_EnableRotation(
    BNO085 *dev,
    uint32_t interval_us
);


BNO_PortStatus BNO085_EnableGameRotation(
    BNO085 *dev,
    uint32_t interval_us
);


/* ============================================================
 * Get latest values
 * ============================================================ */

bool BNO085_GetAccel(
    BNO085 *dev,
    BNO_Vector3 *accel
);


bool BNO085_GetGyro(
    BNO085 *dev,
    BNO_Vector3 *gyro
);


bool BNO085_GetMag(
    BNO085 *dev,
    BNO_Vector3 *mag
);


bool BNO085_GetLinearAccel(
    BNO085 *dev,
    BNO_Vector3 *linear_accel
);


bool BNO085_GetGravity(
    BNO085 *dev,
    BNO_Vector3 *gravity
);


bool BNO085_GetRotation(
    BNO085 *dev,
    BNO_Quaternion *q
);


bool BNO085_GetGameRotation(
    BNO085 *dev,
    BNO_Quaternion *q
);


float BNO085_GetEulerYaw(
    const BNO_Quaternion *q
);


#ifdef __cplusplus
}
#endif

#endif