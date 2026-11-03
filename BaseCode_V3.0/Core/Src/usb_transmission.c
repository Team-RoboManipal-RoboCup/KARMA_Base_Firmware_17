/*
 * usb_transmission.c
 * USB CDC transport for robot telemetry and commands.
 */

#include "USB_transmission.h"

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <string.h>
#include <limits.h>

/* Static transmission buffer */
static uint8_t s_tx_buffer[USB_TRANSMISSION_TX_BUFFER_SIZE];

static volatile bool s_tx_busy = false;


/* ============================================================
 * Get USB CDC handle
 * ============================================================ */

static USBD_CDC_HandleTypeDef *USB_Transmission_GetCdcHandle(void)
{
    if (hUsbDeviceFS.pClassData == NULL)
    {
        return NULL;
    }

    return (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
}


/* ============================================================
 * Initialize USB transmission module
 * ============================================================ */

void USB_Transmission_Init(void)
{
    s_tx_busy = false;

    memset(s_tx_buffer, 0, sizeof(s_tx_buffer));
}


/* ============================================================
 * Transmit arbitrary data
 * ============================================================ */

USB_TransmissionStatus USB_Transmission_Send(
    const uint8_t *data,
    uint16_t length)
{
    USBD_CDC_HandleTypeDef *hcdc;
    uint8_t cdc_status;

    if (data == NULL && length != 0U)
    {
        return USB_TRANSMISSION_INVALID_ARGUMENT;
    }

    if (length == 0U)
    {
        return USB_TRANSMISSION_OK;
    }

    if ((size_t)length > sizeof(s_tx_buffer))
    {
        return USB_TRANSMISSION_TOO_LARGE;
    }

    /* Check whether USB CDC is configured */
    hcdc = USB_Transmission_GetCdcHandle();

    if (hcdc == NULL)
    {
        return USB_TRANSMISSION_BUSY;
    }

    /* Check whether previous transmission is complete */
    if (USB_Transmission_IsBusy())
    {
        return USB_TRANSMISSION_BUSY;
    }

    /* Copy data into transport-owned buffer */
    memcpy(s_tx_buffer, data, length);

    s_tx_busy = true;

    /* Start USB CDC transmission */
    cdc_status = CDC_Transmit_FS(s_tx_buffer, length);

    if (cdc_status == USBD_OK)
    {
        return USB_TRANSMISSION_OK;
    }

    /* Transmission wasn't accepted */
    s_tx_busy = false;

    if (cdc_status == USBD_BUSY)
    {
        return USB_TRANSMISSION_BUSY;
    }

    return USB_TRANSMISSION_ERROR;
}


/* ============================================================
 * Transmit null-terminated string
 * ============================================================ */

USB_TransmissionStatus USB_Transmission_SendString(const char *text)
{
    size_t length;

    if (text == NULL)
    {
        return USB_TRANSMISSION_INVALID_ARGUMENT;
    }

    length = strlen(text);

    if (length > UINT16_MAX ||
        length > USB_TRANSMISSION_TX_BUFFER_SIZE)
    {
        return USB_TRANSMISSION_TOO_LARGE;
    }

    return USB_Transmission_Send(
        (const uint8_t *)text,
        (uint16_t)length
    );
}


/* ============================================================
 * Check transmission status
 * ============================================================ */

bool USB_Transmission_IsBusy(void)
{
    USBD_CDC_HandleTypeDef *hcdc =
        USB_Transmission_GetCdcHandle();

    if (hcdc == NULL)
    {
        s_tx_busy = false;
        return false;
    }

    /* Check STM32 USB CDC transmission state */
    if (hcdc->TxState != 0U)
    {
        return true;
    }

    /* Transfer completed */
    if (s_tx_busy)
    {
        s_tx_busy = false;
    }

    return false;
}


/* ============================================================
 * Transmission completion callback
 * ============================================================ */

void USB_Transmission_TxCompleteCallback(void)
{
    s_tx_busy = false;
}
