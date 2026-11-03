/*
 * CDC-ACM device on the CH32V20x USBFS controller (port 1).
 *
 * The USB controller itself is driven by TinyUSB's USBFS DCD (usbfs_dcd.c),
 * because TinyUSB's device stack can only own one controller. On top of it this
 * file implements just the enumeration the CDC needs plus the CDC class
 * requests and two bulk endpoints, so the second device role is proven code
 * down to the register accesses.
 */

#include "usbfs_cdc.h"

#include "usbfs_mini.h"

#include "class/cdc/cdc.h"

#include <stdio.h>
#include <string.h>

#define RHPORT       1
#define EP0_MPS      64
#define EP_MPS       64

// Safety valve for the serialized IN pump: a frame armed but not completed
// within this time means the host stopped reading (port closed or gone), so the
// pump drops it and resets the endpoint rather than wedging on it forever. A
// healthy read completes in milliseconds, so this never fires in normal use.
#define TX_PENDING_TIMEOUT_MS 1000

#define EP_NOTIFY    0x81
#define EP_DATA_OUT  0x02
#define EP_DATA_IN   0x82  // EP2 IN: EP3's IN path is unreliable on this part, and the DCD
                           // already handles a bidirectional EP2 (RX at +0, TX at +64)

//--------------------------------------------------------------------+
// Descriptors
//--------------------------------------------------------------------+

#define USBFS_VID 0xcafe
#define USBFS_PID 0x4001

static const uint8_t desc_device[] = {
  0x12, 0x01,             // bLength, DEVICE
  0x10, 0x01,             // bcdUSB 1.10
  0x02, 0x00, 0x00,       // class/subclass/protocol: CDC
  EP0_MPS,                // bMaxPacketSize0
  USBFS_VID & 0xff, (USBFS_VID >> 8) & 0xff,
  USBFS_PID & 0xff, (USBFS_PID >> 8) & 0xff,
  0x00, 0x01,             // bcdDevice
  0x01,                   // iManufacturer
  0x02,                   // iProduct
  0x03,                   // iSerialNumber
  0x01,                   // bNumConfigurations
};

static const uint8_t desc_config[] = {
  // Configuration: 2 interfaces, value 1, bus powered, 50 mA
  0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

  // Interface 0: CDC communication, 1 endpoint
  0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
  // Header functional descriptor
  0x05, 0x24, 0x00, 0x10, 0x01,
  // Call management functional descriptor
  0x05, 0x24, 0x01, 0x00, 0x01,
  // ACM functional descriptor
  0x04, 0x24, 0x02, 0x02,
  // Union functional descriptor: interface 0 controls interface 1
  0x05, 0x24, 0x06, 0x00, 0x01,
  // Notification endpoint (interrupt IN, unused)
  0x07, 0x05, EP_NOTIFY, 0x03, 16, 0x00, 0x01,

  // Interface 1: CDC data, 2 endpoints
  0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,
  // Bulk OUT
  0x07, 0x05, EP_DATA_OUT, 0x02, EP_MPS, 0x00, 0x00,
  // Bulk IN
  0x07, 0x05, EP_DATA_IN, 0x02, EP_MPS, 0x00, 0x00,
};

// String descriptors are built at init so bLength cannot drift from the text.
#define STR_DESC_MAX_CHARS 32

static const uint8_t desc_lang[4] = {0x04, 0x03, 0x09, 0x04};  // 0x0409 en-US
static uint8_t desc_manufacturer[2 + 2 * STR_DESC_MAX_CHARS];
static uint8_t desc_product[2 + 2 * STR_DESC_MAX_CHARS];
static uint8_t desc_serial[2 + 2 * STR_DESC_MAX_CHARS];

static void make_string_descriptor(uint8_t *out, const char *text) {
  size_t len = strlen(text);
  if (len > STR_DESC_MAX_CHARS) {
    len = STR_DESC_MAX_CHARS;
  }
  out[0] = (uint8_t) (2 + 2 * len);
  out[1] = 0x03;  // TUSB_DESC_STRING
  for (size_t i = 0; i < len; i++) {
    out[2 + 2 * i] = (uint8_t) text[i];
    out[3 + 2 * i] = 0;
  }
}

//--------------------------------------------------------------------+
// State
//--------------------------------------------------------------------+

static struct {
  bool configured;
  bool suspended;
  uint8_t line_state;      // bit0 DTR, bit1 RTS (both only reported, never gate)
  uint8_t line_coding[7];  // 115200 8N1
  bool tx_busy;
} dev;

// Control transfer being served on EP0
static struct {
  tusb_control_request_t req;
  const uint8_t *in_ptr;
  uint16_t in_remaining;
  bool out_data;           // data stage is host -> device
  bool apply_address;      // SET_ADDRESS: apply once the status stage completed
  uint8_t new_address;
} ctl;

volatile uint32_t usbfs_dbg_tx_arms;
volatile uint32_t usbfs_dbg_tx_done;
volatile uint32_t usbfs_dbg_tx_stuck;
volatile uint32_t usbfs_dbg_tx_timeout;  // safety-valve resets: host stopped reading a pending frame
volatile uint32_t usbfs_dbg_isr_other;

//--------------------------------------------------------------------+
// Rings
//--------------------------------------------------------------------+

#define RX_MASK (USBFS_CDC_RX_RING_SIZE - 1u)
#define TX_MASK (USBFS_CDC_TX_RING_SIZE - 1u)

static uint8_t rx_ring[USBFS_CDC_RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;

static uint8_t tx_ring[USBFS_CDC_TX_RING_SIZE];
static volatile uint16_t tx_head, tx_tail;

static inline uint16_t ring_count(uint16_t head, uint16_t tail, uint16_t mask) {
  return (uint16_t) ((head - tail) & mask);
}

uint32_t usbfs_cdc_available(void) {
  return ring_count(rx_head, rx_tail, RX_MASK);
}

uint32_t usbfs_cdc_read(void *buf, uint32_t len) {
  uint8_t *out = (uint8_t *) buf;
  uint32_t n = 0;
  while (n < len && rx_tail != rx_head) {
    out[n++] = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) & RX_MASK;
  }
  return n;
}

uint32_t usbfs_cdc_write(void const *buf, uint32_t len) {
  const uint8_t *in = (const uint8_t *) buf;
  uint32_t n = 0;
  while (n < len && ring_count(tx_head, tx_tail, TX_MASK) < TX_MASK) {
    tx_ring[tx_head] = in[n++];
    tx_head = (tx_head + 1) & TX_MASK;
  }
  return n;
}

bool usbfs_cdc_connected(void) {
  return dev.configured && (dev.line_state & 0x01);
}

uint8_t usbfs_cdc_line_state(void) {
  return dev.line_state;
}

bool usbfs_cdc_enumerated(void) {
  return dev.configured;
}

//--------------------------------------------------------------------+
// Event queue (controller ISR -> task)
//--------------------------------------------------------------------+

typedef struct {
  dcd_eventid_t id;
  uint8_t ep_addr;
  uint16_t len;
  uint8_t setup[8];
} mini_evt_t;

#define EVT_MAX 16
static mini_evt_t evt_q[EVT_MAX];
static volatile uint8_t evt_head, evt_tail;

static void evt_push(mini_evt_t const *evt) {
  const uint8_t next = (uint8_t) ((evt_head + 1) % EVT_MAX);
  if (next == evt_tail) {
    return;  // queue full: drop, the host retries
  }
  evt_q[evt_head] = *evt;
  evt_head = next;
}

void usbfs_evt_setup_received(uint8_t rhport, const uint8_t *buffer, bool in_isr) {
  (void) rhport;
  (void) in_isr;
  mini_evt_t evt = {.id = DCD_EVENT_SETUP_RECEIVED};
  memcpy(evt.setup, buffer, 8);
  evt_push(&evt);
}

void usbfs_evt_bus_reset(uint8_t rhport, tusb_speed_t speed, bool in_isr) {
  (void) rhport;
  (void) speed;
  (void) in_isr;
  mini_evt_t evt = {.id = DCD_EVENT_BUS_RESET_END};
  evt_push(&evt);
}

void usbfs_evt_bus_signal(uint8_t rhport, dcd_eventid_t event_id, bool in_isr) {
  (void) rhport;
  (void) in_isr;
  mini_evt_t evt = {.id = event_id};
  evt_push(&evt);
}

void usbfs_evt_xfer_complete(uint8_t rhport, uint8_t ep_addr, uint16_t len, xfer_result_t result, bool in_isr) {
  (void) rhport;
  (void) result;
  (void) in_isr;
  mini_evt_t evt = {.id = DCD_EVENT_XFER_COMPLETE, .ep_addr = ep_addr, .len = len};
  evt_push(&evt);
}

//--------------------------------------------------------------------+
// Endpoint buffers and helpers
//--------------------------------------------------------------------+

static uint8_t ep0_out_buf[EP0_MPS];
static uint8_t ep_out_buf[EP_MPS];
static uint8_t ep_in_buf[EP_MPS];

static void ep0_send_next(void) {
  uint16_t n = ctl.in_remaining > EP0_MPS ? EP0_MPS : ctl.in_remaining;
  ctl.in_remaining = (uint16_t) (ctl.in_remaining - n);
  usbfs_dcd_edpt_xfer(RHPORT, 0x80, (uint8_t *) (uintptr_t) ctl.in_ptr, n, false);
  ctl.in_ptr += n;
}

static void ep0_send(const uint8_t *data, uint16_t len) {
  ctl.out_data = false;
  ctl.in_ptr = data;
  ctl.in_remaining = len;
  ep0_send_next();
}

static void ep0_status_in(void) {
  // Status stage for a request without a data stage, or after an OUT data stage
  usbfs_dcd_edpt_xfer(RHPORT, 0x80, NULL, 0, false);
}

static void ep0_stall(void) {
  usbfs_dcd_edpt_stall(RHPORT, 0x00);
  usbfs_dcd_edpt_stall(RHPORT, 0x80);
}

static uint32_t tx_next_ms;

static bool open_data_endpoints(void) {
  tusb_desc_endpoint_t ep = {
    .bLength = sizeof(tusb_desc_endpoint_t),
    .bDescriptorType = TUSB_DESC_ENDPOINT,
    .bEndpointAddress = EP_DATA_OUT,
    .bmAttributes = {.xfer = TUSB_XFER_BULK},
    .wMaxPacketSize = EP_MPS,
    .bInterval = 0,
  };
  if (!usbfs_dcd_edpt_open(RHPORT, &ep)) {
    return false;
  }
  ep.bEndpointAddress = EP_DATA_IN;
  if (!usbfs_dcd_edpt_open(RHPORT, &ep)) {
    return false;
  }

  // Notification endpoint: opened so the host's interrupt IN polls are answered
  // (a stalled notification endpoint upsets cdc_acm); we never queue data on it.
  ep.bEndpointAddress = EP_NOTIFY;
  ep.bmAttributes.xfer = TUSB_XFER_INTERRUPT;
  ep.wMaxPacketSize = 16;
  ep.bInterval = 1;
  if (!usbfs_dcd_edpt_open(RHPORT, &ep)) {
    return false;
  }

  // Start the first OUT transfer; it is re-armed after every completion
  return usbfs_dcd_edpt_xfer(RHPORT, EP_DATA_OUT, ep_out_buf, EP_MPS, false);
}

//--------------------------------------------------------------------+
// Control transfers
//--------------------------------------------------------------------+

static const uint8_t *string_descriptor(uint8_t index, uint16_t *len) {
  switch (index) {
    case 0: *len = sizeof(desc_lang);         return desc_lang;
    case 1: *len = desc_manufacturer[0];      return desc_manufacturer;
    case 2: *len = desc_product[0];           return desc_product;
    case 3: *len = desc_serial[0];            return desc_serial;
    default: *len = 0;                        return NULL;
  }
}

static void handle_setup(uint8_t const *setup) {
  tusb_control_request_t req;
  memcpy(&req, setup, sizeof(req));
  ctl.req = req;

  if (req.bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
    switch (req.bRequest) {
      case TUSB_REQ_GET_DESCRIPTOR: {
        const uint8_t type = (uint8_t) (req.wValue >> 8);
        const uint8_t *desc = NULL;
        uint16_t len = 0;
        if (type == TUSB_DESC_DEVICE) {
          desc = desc_device;
          len = sizeof(desc_device);
        } else if (type == TUSB_DESC_CONFIGURATION) {
          desc = desc_config;
          len = sizeof(desc_config);
        } else if (type == TUSB_DESC_STRING) {
          desc = string_descriptor((uint8_t) (req.wValue & 0xff), &len);
        }
        if (desc == NULL) {
          ep0_stall();
          return;
        }
        if (len > req.wLength) {
          len = req.wLength;
        }
        ep0_send(desc, len);
        return;
      }

      case TUSB_REQ_SET_ADDRESS:
        // The DCD sends the status ZLP here; the address is written from
        // usbfs_dcd_edpt0_status_complete() once that stage completed.
        ctl.apply_address = true;
        ctl.new_address   = (uint8_t) (req.wValue & 0xff);
        usbfs_dcd_set_address(RHPORT, ctl.new_address);
        return;

      case TUSB_REQ_SET_CONFIGURATION: {
        const uint8_t cfg = (uint8_t) (req.wValue & 0xff);
        if (cfg == 0) {
          dev.configured = false;
          usbfs_dcd_edpt_close_all(RHPORT);
        } else {
          dev.configured = open_data_endpoints();
        }
        ep0_status_in();
        return;
      }

      case TUSB_REQ_GET_CONFIGURATION: {
        static uint8_t cfg = 1;
        ep0_send(&cfg, 1);
        return;
      }

      case TUSB_REQ_GET_STATUS: {
        static uint8_t status[2] = {0, 0};
        ep0_send(status, 2);
        return;
      }

      case TUSB_REQ_GET_INTERFACE: {
        static uint8_t alt = 0;
        ep0_send(&alt, 1);
        return;
      }

      case TUSB_REQ_SET_INTERFACE:
      case TUSB_REQ_CLEAR_FEATURE:
      case TUSB_REQ_SET_FEATURE:
        ep0_status_in();
        return;

      default:
        ep0_stall();
        return;
    }
  } else if (req.bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
    switch (req.bRequest) {
      case CDC_REQUEST_SET_LINE_CODING:  // 7 bytes, host -> device
        ctl.out_data = true;
        usbfs_dcd_edpt_xfer(RHPORT, 0x00, ep0_out_buf, req.wLength, false);
        return;

      case CDC_REQUEST_GET_LINE_CODING:
        ep0_send(dev.line_coding, sizeof(dev.line_coding));
        return;

      case CDC_REQUEST_SET_CONTROL_LINE_STATE: {
        const bool was_open = (dev.line_state & 0x01) != 0;
        dev.line_state      = (uint8_t) (req.wValue & 0x03);
        const bool is_open  = (dev.line_state & 0x01) != 0;

        if (was_open && !is_open) {
          // Host closed the port: a frame it never polled still holds the IN
          // endpoint, and would refuse every later transfer.
          // Only the IN endpoint can wedge on our side; the OUT endpoint is
          // host driven and must stay armed.
          usbfs_dcd_edpt_reset(RHPORT, EP_DATA_IN);
          dev.tx_busy = false;
        } else if (!was_open && is_open) {
          // Fresh session: make both directions usable again.
          usbfs_dcd_edpt_reset(RHPORT, EP_DATA_IN);
          dev.tx_busy = false;
          usbfs_dcd_edpt_xfer(RHPORT, EP_DATA_OUT, ep_out_buf, EP_MPS, false);
        }
        ep0_status_in();
        return;
      }

      default:
        ep0_stall();
        return;
    }
  } else {
    ep0_stall();
  }
}

static void handle_ep0_complete(uint8_t ep_addr, uint16_t len) {
  if (ep_addr == 0x80) {
    if (ctl.in_remaining > 0) {
      ep0_send_next();
    } else if (ctl.apply_address) {
      ctl.apply_address = false;
      usbfs_dcd_edpt0_status_complete(RHPORT, &ctl.req);
    } else if (!ctl.out_data) {
      // IN data stage done: accept the OUT status stage
      usbfs_dcd_edpt_xfer(RHPORT, 0x00, NULL, 0, false);
    }
  } else {  // 0x00: OUT data stage (SET_LINE_CODING) or the status stage
    if (ctl.out_data) {
      if (len >= sizeof(dev.line_coding)) {
        memcpy(dev.line_coding, ep0_out_buf, sizeof(dev.line_coding));
      }
      ctl.out_data = false;
      ep0_status_in();
    }
  }
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

void usbfs_cdc_init(void) {
  memset(&dev, 0, sizeof(dev));
  memset(&ctl, 0, sizeof(ctl));
  dev.line_coding[0] = 0x00;  // 115200
  dev.line_coding[1] = 0xc2;
  dev.line_coding[2] = 0x01;
  dev.line_coding[3] = 0x00;
  dev.line_coding[4] = 0x00;  // 1 stop bit
  dev.line_coding[5] = 0x00;  // no parity
  dev.line_coding[6] = 0x08;  // 8 data bits

  evt_head = evt_tail = 0;
  rx_head = rx_tail = 0;
  tx_head = tx_tail = 0;

  // Keep the two devices distinguishable in lsusb / device manager.
  make_string_descriptor(desc_manufacturer, "TinyUSB");
  make_string_descriptor(desc_product, "CDC Bridge");
  make_string_descriptor(desc_serial, "203C10002");

  const tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
  usbfs_dcd_init(RHPORT, &init);
  usbfs_dcd_int_enable(RHPORT);
}

void usbfs_cdc_task(void) {
  while (evt_tail != evt_head) {
    const mini_evt_t evt = evt_q[evt_tail];
    evt_tail = (uint8_t) ((evt_tail + 1) % EVT_MAX);

    switch (evt.id) {
      case DCD_EVENT_SETUP_RECEIVED:
        handle_setup(evt.setup);
        break;

      case DCD_EVENT_XFER_COMPLETE:
        if (evt.ep_addr == 0x00 || evt.ep_addr == 0x80) {
          handle_ep0_complete(evt.ep_addr, evt.len);
        } else if (evt.ep_addr == EP_DATA_OUT) {
          for (uint16_t i = 0; i < evt.len; i++) {
            const uint16_t next = (uint16_t) ((rx_head + 1) & RX_MASK);
            if (next == rx_tail) {
              break;  // ring full: drop the tail of this packet
            }
            rx_ring[rx_head] = ep_out_buf[i];
            rx_head = next;
          }
          usbfs_dcd_edpt_xfer(RHPORT, EP_DATA_OUT, ep_out_buf, EP_MPS, false);
        } else if (evt.ep_addr == EP_DATA_IN) {
          // The armed IN frame left the controller. This completion is what
          // normally releases the endpoint for the next frame (the pump below
          // stays serialized on it); the safety valve is only a fallback.
          dev.tx_busy = false;
          usbfs_dbg_tx_done++;
        }
        break;

      case DCD_EVENT_BUS_RESET_END:
        dev.configured = false;
        dev.suspended = false;
        dev.tx_busy = false;
        dev.line_state = 0;
        ctl.in_remaining = 0;
        break;

      case DCD_EVENT_SUSPEND:
        dev.suspended = true;
        break;

      case DCD_EVENT_RESUME:
        dev.suspended = false;
        break;

      default:
        usbfs_dbg_isr_other++;
        break;
    }
  }

  // Transmit pump -- serialized on the controller's completion, like usbd's
  // claim/complete model. The IN endpoint is armed once and must not be touched
  // again until DCD_EVENT_XFER_COMPLETE (handled above) clears tx_busy. The old
  // fire-and-forget pump re-armed on a 10 ms timer regardless, overwriting the
  // DMA buffer from task context while the host could still be mid-read of the
  // previous frame -- the race that wedged the channel (measured on hardware:
  // arms=292, done=0 while the host was not reading). The safety valve below
  // only covers a host that stops reading entirely.
  if (!dev.configured) {
    return;
  }

  const uint32_t now = tusb_time_millis_api();
  if (dev.tx_busy) {
    if ((int32_t) (now - tx_next_ms) < TX_PENDING_TIMEOUT_MS) {
      return;  // previous frame still in flight: never re-arm over a live transfer
    }
    // No completion within the timeout: the host has stopped reading (port
    // closed or gone). Drop the frame and reset the endpoint so it is not left
    // armed, then re-sync on the next frame instead of wedging forever.
    usbfs_dcd_edpt_reset(RHPORT, EP_DATA_IN);
    dev.tx_busy = false;
    usbfs_dbg_tx_timeout++;
  }

  if (tx_tail == tx_head) {
    return;
  }

  uint16_t n = ring_count(tx_head, tx_tail, TX_MASK);
  if (n > EP_MPS) {
    n = EP_MPS;
  }
  for (uint16_t i = 0; i < n; i++) {
    ep_in_buf[i] = tx_ring[(tx_tail + i) & TX_MASK];
  }

  if (usbfs_dcd_edpt_xfer(RHPORT, EP_DATA_IN, ep_in_buf, n, false)) {
    tx_tail    = (uint16_t) ((tx_tail + n) & TX_MASK);
    dev.tx_busy = true;
    tx_next_ms  = now;  // arm timestamp: the safety valve measures from here
    usbfs_dbg_tx_arms++;
  } else {
    usbfs_dbg_tx_stuck++;
  }
}

// Interrupt hook installed over the weak usbfs_isr_hook() in the BSP.
void usbfs_isr_hook(void) {
  usbfs_dcd_int_handler(RHPORT);
}
