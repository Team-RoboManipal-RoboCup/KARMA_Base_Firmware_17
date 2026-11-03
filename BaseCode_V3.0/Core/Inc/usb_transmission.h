/*
 * USB_transmission.h
 * USB CDC transmission interface.
 */

#ifndef INC_DRIVERS_USB_TRANSMISSION_H_
#define INC_DRIVERS_USB_TRANSMISSION_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_TRANSMISSION_TX_BUFFER_SIZE 512U

typedef enum
{
    USB_TRANSMISSION_OK = 0,
    USB_TRANSMISSION_BUSY,
    USB_TRANSMISSION_ERROR,
    USB_TRANSMISSION_INVALID_ARGUMENT,
    USB_TRANSMISSION_TOO_LARGE

} USB_TransmissionStatus;


/* Initialize transport */
void USB_Transmission_Init(void);


/* Transmit arbitrary data */
USB_TransmissionStatus USB_Transmission_Send(
    const uint8_t *data,
    uint16_t length
);


/* Transmit a null-terminated string */
USB_TransmissionStatus USB_Transmission_SendString(
    const char *text
);


/* Check transmission status */
bool USB_Transmission_IsBusy(void);


/* Called when USB CDC transmission completes */
void USB_Transmission_TxCompleteCallback(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_DRIVERS_USB_TRANSMISSION_H_ */
