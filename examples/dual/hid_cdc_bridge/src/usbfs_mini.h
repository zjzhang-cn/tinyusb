/*
 * Minimal device stack for the CH32V20x USBFS controller.
 *
 * TinyUSB's device stack can only drive one controller, so the CDC role on this
 * port uses TinyUSB's USBFS controller driver (usbfs_dcd.c, copied from
 * src/portable/wch) with just enough enumeration and CDC-ACM logic on top.
 *
 * The DCD reports through usbfs_evt_*() into a small queue that usbfs_cdc_task()
 * drains, so no USB work happens in the interrupt beyond the register handling
 * the DCD already does.
 */

#ifndef USBFS_MINI_H_
#define USBFS_MINI_H_

#include <stdint.h>
#include <stdbool.h>

#include "tusb.h"
#include "device/dcd.h"

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Controller driver (usbfs_dcd.c, copied from src/portable/wch/dcd_ch32_usbfs.c)
//--------------------------------------------------------------------+

bool usbfs_dcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init);
void usbfs_dcd_int_handler(uint8_t rhport);
void usbfs_dcd_int_enable(uint8_t rhport);
void usbfs_dcd_int_disable(uint8_t rhport);
void usbfs_dcd_set_address(uint8_t rhport, uint8_t dev_addr);
void usbfs_dcd_remote_wakeup(uint8_t rhport);
void usbfs_dcd_connect(uint8_t rhport);
void usbfs_dcd_disconnect(uint8_t rhport);
void usbfs_dcd_sof_enable(uint8_t rhport, bool en);
bool usbfs_dcd_edpt_open(uint8_t rhport, const tusb_desc_endpoint_t *desc_ep);
void usbfs_dcd_edpt_close_all(uint8_t rhport);
void usbfs_dcd_edpt_reset(uint8_t rhport, uint8_t ep_addr);
bool usbfs_dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size);
bool usbfs_dcd_edpt_iso_activate(uint8_t rhport, const tusb_desc_endpoint_t *desc_ep);
bool usbfs_dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes, bool is_isr);
void usbfs_dcd_edpt0_status_complete(uint8_t rhport, const tusb_control_request_t *request);
void usbfs_dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr);
void usbfs_dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr);

//--------------------------------------------------------------------+
// Events from the controller driver to the mini stack
//--------------------------------------------------------------------+

void usbfs_evt_setup_received(uint8_t rhport, const uint8_t *buffer, bool in_isr);
void usbfs_evt_bus_reset(uint8_t rhport, tusb_speed_t speed, bool in_isr);
void usbfs_evt_bus_signal(uint8_t rhport, dcd_eventid_t event_id, bool in_isr);
void usbfs_evt_xfer_complete(uint8_t rhport, uint8_t ep_addr, uint16_t len, xfer_result_t result, bool in_isr);

#ifdef __cplusplus
}
#endif

#endif /* USBFS_MINI_H_ */
