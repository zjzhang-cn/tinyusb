/*
 * CDC-ACM device on the second USB controller (USBFS / port 1) of CH32V20x.
 *
 * TinyUSB's device stack is single-instance, so the second device port cannot
 * be served by it. This is a trimmed version of the WCH USBFS device driver
 * (hw/mcu/wch/ch32v20x/EVT/EXAM/USB/USBFS/DEVICE/SimulateCDC) with the UART
 * pass-through removed and a pair of ring buffers exposed instead.
 */

#ifndef USBFS_CDC_H_
#define USBFS_CDC_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USBFS_CDC_RX_RING_SIZE 256
#define USBFS_CDC_TX_RING_SIZE 128

void usbfs_cdc_init(void);

// Pump the transmit ring. Call from the main loop.
void usbfs_cdc_task(void);

// Bytes waiting in the receive ring.
uint32_t usbfs_cdc_available(void);

// Read up to len bytes from the receive ring.
uint32_t usbfs_cdc_read(void *buf, uint32_t len);

// Queue bytes for the host. Returns the number actually queued.
uint32_t usbfs_cdc_write(const void *buf, uint32_t len);

// True when the host has the port open (DTR asserted).
bool usbfs_cdc_connected(void);

// True when the device is enumerated (configured), whether or not the host has
// the port open.
bool usbfs_cdc_enumerated(void);

// Interrupt hook installed over the weak usbfs_isr_hook() in the BSP.
void usbfs_isr_hook(void);

#ifdef __cplusplus
}
#endif

#endif /* USBFS_CDC_H_ */
