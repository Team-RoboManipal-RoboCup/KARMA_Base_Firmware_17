#ifndef BNO_PORT_H
#define BNO_PORT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Return codes
 * -------------------------------------------------------------------------- */

typedef enum
{
    BNO_PORT_OK = 0,
    BNO_PORT_ERROR,
    BNO_PORT_TIMEOUT

} BNO_PortStatus;


/* --------------------------------------------------------------------------
 * Hardware interface
 *
 * The BNO085 driver uses these functions without knowing whether the
 * underlying hardware is STM32 HAL, STM32 LL, ESP32, RP2040, etc.
 * -------------------------------------------------------------------------- */

typedef struct
{
    BNO_PortStatus (*write)(const uint8_t *data,
                            size_t length);

    BNO_PortStatus (*read)(uint8_t *data,
                           size_t length);

    void (*delay_ms)(uint32_t ms);

    void (*reset)(void);

} BNO_Port;



#include <stdbool.h>

void BNO_Port_Init(BNO_Port *port);
bool BNO_Port_Detect(void);
uint8_t BNO_Port_GetAddress(void);


/* -------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* BNO_PORT_H */

