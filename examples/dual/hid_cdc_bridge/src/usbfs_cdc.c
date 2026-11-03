/*
 * CDC-ACM device on the USBFS controller (port 1).
 *
 * Derived from the WCH EVT example "USBFS/DEVICE/SimulateCDC"
 * (Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.,
 *  Apache-2.0 / WCH EVT license). The UART bridging has been replaced by
 * ring buffers; keep the register sequences as-is when editing.
 */

#include "usbfs_cdc.h"

// The SDK headers are not prototype-clean; match the BSP's pragmas.
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif

#include "ch32v20x.h"
#include "ch32v20x_usb.h"

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif

#include <string.h>

//--------------------------------------------------------------------+
// USBFS register helpers (same layout as the WCH driver)
//--------------------------------------------------------------------+

#define USBFSD_UEP_MOD_BASE 0x5000000C
#define USBFSD_UEP_DMA_BASE 0x50000010
#define USBFSD_UEP_LEN_BASE 0x50000030
#define USBFSD_UEP_CTL_BASE 0x50000032

#define USBFSD_UEP_RX_EN    0x08
#define USBFSD_UEP_TX_EN    0x04
#define USBFSD_UEP_BUF_MOD  0x01

#define USBFSD_UEP_MOD(n)   (*((volatile uint8_t *)(USBFSD_UEP_MOD_BASE + (n))))
#define USBFSD_UEP_CTRL(n)  (*((volatile uint8_t *)(USBFSD_UEP_CTL_BASE + (n) * 0x04)))
#define USBFSD_UEP_DMA(n)   (*((volatile uint32_t *)(USBFSD_UEP_DMA_BASE + (n) * 0x04)))
#define USBFSD_UEP_BUF(n)   ((uint8_t *)(USBFSD_UEP_DMA(n) + 0x20000000))
#define USBFSD_UEP_TLEN(n)  (*((volatile uint16_t *)(USBFSD_UEP_LEN_BASE + (n) * 0x04)))

#define DEF_UEP_IN          0x80
#define DEF_UEP_OUT         0x00
#define DEF_UEP0            0x00
#define DEF_UEP1            0x01
#define DEF_UEP2            0x02
#define DEF_UEP3            0x03
#define DEF_UEP_NUM         8

#define EP0_SIZE            64
#define EP1_SIZE            16
#define EP2_SIZE            64
#define EP3_SIZE            64

// Endpoint layout: EP1 IN = CDC notification (unused), EP2 OUT = data from
// host, EP3 IN = data to host.
#define EP_NOTIFY   1
#define EP_DATA_OUT 2
#define EP_DATA_IN  3

//--------------------------------------------------------------------+
// Descriptors
//--------------------------------------------------------------------+

#define USBFS_VID 0xcafe
#define USBFS_PID 0x4001

static const uint8_t desc_device[] = {
  0x12,
  0x01,     // bLength, DEVICE
  0x10,
  0x01,     // bcdUSB 1.10
  0x02,
  0x00,
  0x00,     // class/subclass/protocol: CDC
  EP0_SIZE, // bMaxPacketSize0
  USBFS_VID & 0xff,
  (USBFS_VID >> 8) & 0xff,
  USBFS_PID & 0xff,
  (USBFS_PID >> 8) & 0xff,
  0x00,
  0x01, // bcdDevice
  0x01, // iManufacturer
  0x02, // iProduct
  0x03, // iSerialNumber
  0x01, // bNumConfigurations
};

static const uint8_t desc_config[] = {
  // Configuration: 2 interfaces, 1 = bConfigurationValue, 0x80 = bus powered, 50 mA
  0x09,
  0x02,
  0x43,
  0x00,
  0x02,
  0x01,
  0x00,
  0x80,
  0x32,

  // Interface 0: CDC communication, 1 endpoint
  0x09,
  0x04,
  0x00,
  0x00,
  0x01,
  0x02,
  0x02,
  0x01,
  0x00,
  // Header functional descriptor
  0x05,
  0x24,
  0x00,
  0x10,
  0x01,
  // Call management functional descriptor (no call management)
  0x05,
  0x24,
  0x01,
  0x00,
  0x01,
  // ACM functional descriptor (line coding, serial state)
  0x04,
  0x24,
  0x02,
  0x02,
  // Union functional descriptor: interface 0 controls interface 1
  0x05,
  0x24,
  0x06,
  0x00,
  0x01,
  // Notification endpoint (interrupt IN, unused)
  0x07,
  0x05,
  DEF_UEP_IN | EP_NOTIFY,
  0x03,
  EP1_SIZE,
  0x00,
  0x01,

  // Interface 1: CDC data, 2 endpoints
  0x09,
  0x04,
  0x01,
  0x00,
  0x02,
  0x0a,
  0x00,
  0x00,
  0x00,
  // Bulk OUT
  0x07,
  0x05,
  DEF_UEP_OUT | EP_DATA_OUT,
  0x02,
  EP2_SIZE,
  0x00,
  0x00,
  // Bulk IN
  0x07,
  0x05,
  DEF_UEP_IN | EP_DATA_IN,
  0x02,
  EP3_SIZE,
  0x00,
  0x00,
};

static const uint8_t desc_lang[4] = {0x04, 0x03, 0x09, 0x04}; // 0x0409 en-US

// String descriptors are built at init so bLength cannot drift from the text.
// (The WCH example hard-codes bLength in the array: editing the text without
// editing the length silently truncates the string or reads past the array.)
#define STR_DESC_MAX_CHARS 32

static uint8_t desc_manufacturer[2 + 2 * STR_DESC_MAX_CHARS];
static uint8_t desc_product[2 + 2 * STR_DESC_MAX_CHARS];
static uint8_t desc_serial[2 + 2 * STR_DESC_MAX_CHARS];

#define DESC_LEN(desc) ((uint16_t)(desc)[0])

static void make_string_descriptor(uint8_t *out, const char *text) {
  size_t len = strlen(text);
  if (len > STR_DESC_MAX_CHARS) {
    len = STR_DESC_MAX_CHARS;
  }
  out[0] = (uint8_t)(2 + 2 * len);
  out[1] = 0x03; // TUSB_DESC_STRING
  for (size_t i = 0; i < len; i++) {
    out[2 + 2 * i] = (uint8_t)text[i];
    out[3 + 2 * i] = 0;
  }
}

static const uint8_t *desc_ptr;

// TEMP diagnostics: [0]=setups [1]=stalls [2]=last type|code [3]=last wValue [4]=last wIndex
volatile uint32_t dbg[8];

//--------------------------------------------------------------------+
// Driver state
//--------------------------------------------------------------------+

static volatile uint8_t  setup_req_type;
static volatile uint8_t  setup_req_code;
static volatile uint16_t setup_req_value;
static volatile uint16_t setup_req_index;
static volatile uint16_t setup_req_len;

static volatile uint8_t dev_config;
static volatile uint8_t dev_addr;
static volatile uint8_t dev_sleep;
static volatile uint8_t dev_enum;

static uint8_t          line_coding[7] = {0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08}; // 115200 8N1
static volatile uint8_t line_state;                                                  // bit0 DTR, bit1 RTS

__attribute__((aligned(4))) static uint8_t ep0_buf[EP0_SIZE];
__attribute__((aligned(4))) static uint8_t ep1_buf[EP1_SIZE];
__attribute__((aligned(4))) static uint8_t ep2_buf[EP2_SIZE];
__attribute__((aligned(4))) static uint8_t ep3_buf[EP3_SIZE];

static volatile uint8_t ep_busy[DEF_UEP_NUM];

static uint8_t           rx_ring[USBFS_CDC_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool     rx_naked;

static uint8_t           tx_ring[USBFS_CDC_TX_RING_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;

static void usbfs_delay_us(uint32_t us) {
  volatile uint32_t loops = (SystemCoreClock / 1000000U / 4U + 1U) * us;
  while (loops--) {
    __asm volatile("nop");
  }
}

//--------------------------------------------------------------------+
// Ring buffers
//--------------------------------------------------------------------+

#define RX_MASK (USBFS_CDC_RX_RING_SIZE - 1u)
#define TX_MASK (USBFS_CDC_TX_RING_SIZE - 1u)

static inline uint16_t ring_count(uint16_t head, uint16_t tail, uint16_t mask) {
  return (uint16_t)((head - tail) & mask);
}

uint32_t usbfs_cdc_available(void) {
  return ring_count(rx_head, rx_tail, RX_MASK);
}

uint32_t usbfs_cdc_read(void *buf, uint32_t len) {
  uint8_t *out = (uint8_t *)buf;
  uint32_t n   = 0;
  while (n < len && rx_tail != rx_head) {
    out[n++] = rx_ring[rx_tail];
    rx_tail  = (rx_tail + 1) & RX_MASK;
  }
  // Resume receiving if we had throttled the endpoint.
  if (rx_naked && ring_count(rx_head, rx_tail, RX_MASK) < (USBFS_CDC_RX_RING_SIZE / 2)) {
    rx_naked             = false;
    USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
  }
  return n;
}

uint32_t usbfs_cdc_write(const void *buf, uint32_t len) {
  const uint8_t *in = (const uint8_t *)buf;
  uint32_t       n  = 0;
  while (n < len && ring_count(tx_head, tx_tail, TX_MASK) < TX_MASK) {
    tx_ring[tx_head] = in[n++];
    tx_head          = (tx_head + 1) & TX_MASK;
  }
  return n;
}

bool usbfs_cdc_connected(void) {
  return dev_enum && (line_state & 0x01);
}

bool usbfs_cdc_enumerated(void) {
  return dev_enum;
}

//--------------------------------------------------------------------+
// Endpoints
//--------------------------------------------------------------------+

static void usbfs_ep_init(void) {
  USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
  USBFSD->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP3_TX_EN;

  USBFSD->UEP0_DMA = (uint32_t)ep0_buf;
  USBFSD->UEP1_DMA = (uint32_t)ep1_buf;
  USBFSD->UEP2_DMA = (uint32_t)ep2_buf;
  USBFSD->UEP3_DMA = (uint32_t)ep3_buf;

  USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
  USBFSD->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;

  USBFSD->UEP1_TX_LEN = 0;
  USBFSD->UEP3_TX_LEN = 0;

  USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
  USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
  USBFSD->UEP3_TX_CTRL = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;

  for (uint8_t i = 0; i < DEF_UEP_NUM; i++) {
    ep_busy[i] = 0;
  }
}

static void usbfs_device_reset_state(void) {
  dev_config = 0;
  dev_addr   = 0;
  dev_sleep  = 0;
  dev_enum   = 0;
  line_state = 0;
  rx_head = rx_tail = 0;
  tx_head = tx_tail   = 0;
  rx_naked            = false;
  ep_busy[EP_DATA_IN] = 0;
}

static void usbfs_ep_start_tx(uint8_t endp, const uint8_t *buf, uint16_t len) {
  if (ep_busy[endp]) {
    return;
  }
  memcpy(USBFSD_UEP_BUF(endp), buf, len);
  ep_busy[endp]         = 1;
  USBFSD_UEP_TLEN(endp) = len;
  USBFSD_UEP_CTRL(endp) = (USBFSD_UEP_CTRL(endp) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

void usbfs_cdc_init(void) {
  usbfs_device_reset_state();

  // Keep the two devices distinguishable in lsusb / device manager: this port
  // is the control channel, port 0 is the HID one.
  make_string_descriptor(desc_manufacturer, "TinyUSB");
  make_string_descriptor(desc_product, "CDC Bridge");
  make_string_descriptor(desc_serial, "203C10002");

  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  usbfs_delay_us(10);
  USBFSH->BASE_CTRL = 0x00;

  USBFSD->INT_EN    = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  usbfs_ep_init();
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;

  NVIC_EnableIRQ(USBFS_IRQn);
}

void usbfs_cdc_task(void) {
  if (ep_busy[EP_DATA_IN]) {
    return;
  }

  const uint16_t count = ring_count(tx_head, tx_tail, TX_MASK);
  if (count == 0) {
    return;
  }

  uint8_t  chunk[EP3_SIZE];
  uint16_t n = count > EP3_SIZE ? EP3_SIZE : count;
  for (uint16_t i = 0; i < n; i++) {
    chunk[i] = tx_ring[(tx_tail + i) & TX_MASK];
  }

  __disable_irq();
  usbfs_ep_start_tx(EP_DATA_IN, chunk, n);
  __enable_irq();

  tx_tail = (tx_tail + n) & TX_MASK;
}

//--------------------------------------------------------------------+
// Interrupt handler
//--------------------------------------------------------------------+

static void usbfs_setup_stage(void) {
  USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK;
  USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

  const PUSB_SETUP_REQ req = (PUSB_SETUP_REQ)ep0_buf;
  setup_req_type           = req->bRequestType;
  setup_req_code           = req->bRequest;
  setup_req_len            = req->wLength;
  setup_req_value          = req->wValue;
  setup_req_index          = req->wIndex;

  uint16_t len     = 0;
  uint8_t  errflag = 0;

  if ((setup_req_type & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
    if (setup_req_type & USB_REQ_TYP_CLASS) {
      switch (setup_req_code) {
        case CDC_GET_LINE_CODING:
          desc_ptr = line_coding;
          len      = sizeof(line_coding);
          break;

        case CDC_SET_LINE_CODING: // data arrives in the OUT stage
          break;

        case CDC_SET_LINE_CTLSTE: // wValue bit0 = DTR, bit1 = RTS
          line_state = (uint8_t) (setup_req_value & 0x03);
          break;

        case CDC_SEND_BREAK:
          break;

        default:
          errflag = 0xff;
          break;
      }
    } else {
      errflag = 0xff;
    }

    if (errflag == 0) {
      if (setup_req_len > len) {
        setup_req_len = len;
      }
      len = (setup_req_len >= EP0_SIZE) ? EP0_SIZE : setup_req_len;
      if (len > 0) {
        memcpy(ep0_buf, desc_ptr, len);
        desc_ptr += len;
      }
    }
  } else {
    switch (setup_req_code) {
      case USB_GET_DESCRIPTOR:
        switch ((uint8_t)(setup_req_value >> 8)) {
          case USB_DESCR_TYP_DEVICE:
            desc_ptr = desc_device;
            len      = sizeof(desc_device);
            break;

          case USB_DESCR_TYP_CONFIG:
            desc_ptr = desc_config;
            len      = sizeof(desc_config);
            break;

          case USB_DESCR_TYP_STRING:
            switch ((uint8_t)(setup_req_value & 0xff)) {
              case 0:
                desc_ptr = desc_lang;
                len      = sizeof(desc_lang);
                break;
              case 1:
                desc_ptr = desc_manufacturer;
                len      = DESC_LEN(desc_manufacturer);
                break;
              case 2:
                desc_ptr = desc_product;
                len      = DESC_LEN(desc_product);
                break;
              case 3:
                desc_ptr = desc_serial;
                len      = DESC_LEN(desc_serial);
                break;
              default:
                errflag = 0xff;
                break;
            }
            break;

          default:
            errflag = 0xff;
            break;
        }

        if (errflag == 0) {
          if (setup_req_len > len) {
            setup_req_len = len;
          }
          len = (setup_req_len >= EP0_SIZE) ? EP0_SIZE : setup_req_len;
          memcpy(ep0_buf, desc_ptr, len);
          desc_ptr += len;
        }
        break;

      case USB_SET_ADDRESS:
        dev_addr = (uint8_t)(setup_req_value & 0xff);
        break;

      case USB_GET_CONFIGURATION:
        ep0_buf[0] = dev_config;
        if (setup_req_len > 1) {
          setup_req_len = 1;
        }
        break;

      case USB_SET_CONFIGURATION:
        dev_config = (uint8_t)(setup_req_value & 0xff);
        dev_enum   = 1;
        break;

      case USB_CLEAR_FEATURE:
        if ((setup_req_type & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
          if ((setup_req_value & 0xff) == USB_REQ_FEAT_ENDP_HALT) {
            switch ((uint8_t)(setup_req_index & 0xff)) {
              case (DEF_UEP_IN | EP_NOTIFY):
                USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
                break;
              case (DEF_UEP_OUT | EP_DATA_OUT):
                USBFSD->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;
                break;
              case (DEF_UEP_IN | EP_DATA_IN):
                USBFSD->UEP3_TX_CTRL = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
                break;
              default:
                errflag = 0xff;
                break;
            }
          } else if ((setup_req_value & 0xff) == USB_REQ_FEAT_REMOTE_WAKEUP) {
            dev_sleep &= (uint8_t)~0x01;
          } else {
            errflag = 0xff;
          }
        } else if ((setup_req_type & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
          if ((setup_req_value & 0xff) == USB_REQ_FEAT_REMOTE_WAKEUP) {
            dev_sleep &= (uint8_t)~0x01;
          } else {
            errflag = 0xff;
          }
        } else {
          errflag = 0xff;
        }
        break;

      case USB_SET_FEATURE:
        if ((setup_req_type & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
          if ((setup_req_value & 0xff) == USB_REQ_FEAT_ENDP_HALT) {
            switch ((uint8_t)(setup_req_index & 0xff)) {
              case (DEF_UEP_IN | EP_NOTIFY):
                USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                break;
              case (DEF_UEP_OUT | EP_DATA_OUT):
                USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_STALL;
                break;
              case (DEF_UEP_IN | EP_DATA_IN):
                USBFSD->UEP3_TX_CTRL = (USBFSD->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                break;
              default:
                errflag = 0xff;
                break;
            }
          } else {
            errflag = 0xff;
          }
        } else if ((setup_req_type & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
          if ((setup_req_value & 0xff) == USB_REQ_FEAT_REMOTE_WAKEUP) {
            dev_sleep |= 0x01;
          } else {
            errflag = 0xff;
          }
        } else {
          errflag = 0xff;
        }
        break;

      case USB_GET_INTERFACE:
        ep0_buf[0] = 0x00;
        if (setup_req_len > 1) {
          setup_req_len = 1;
        }
        break;

      case USB_SET_INTERFACE:
        break;

      case USB_GET_STATUS:
        ep0_buf[0] = 0x00;
        ep0_buf[1] = 0x00;
        if ((setup_req_type & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
          switch ((uint8_t)(setup_req_index & 0xff)) {
            case (DEF_UEP_IN | EP_NOTIFY):
              if ((USBFSD->UEP1_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) {
                ep0_buf[0] = 0x01;
              }
              break;
            case (DEF_UEP_OUT | EP_DATA_OUT):
              if ((USBFSD->UEP2_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL) {
                ep0_buf[0] = 0x01;
              }
              break;
            case (DEF_UEP_IN | EP_DATA_IN):
              if ((USBFSD->UEP3_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) {
                ep0_buf[0] = 0x01;
              }
              break;
            default:
              errflag = 0xff;
              break;
          }
        } else if ((setup_req_type & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
          if (dev_sleep & 0x01) {
            ep0_buf[0] = 0x02;
          }
        }
        if (setup_req_len > 2) {
          setup_req_len = 2;
        }
        break;

      default:
        errflag = 0xff;
        break;
    }
  }

  if (errflag == 0xff) {
    dbg[1]++;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
    return;
  }

  if (setup_req_type & DEF_UEP_IN) {
    len = (setup_req_len > EP0_SIZE) ? EP0_SIZE : setup_req_len;
    setup_req_len -= len;
    USBFSD->UEP0_TX_LEN  = len;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
  } else {
    if (setup_req_len == 0) {
      USBFSD->UEP0_TX_LEN  = 0;
      USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
    } else {
      USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
    }
  }
}

static void usbfs_in_stage(uint8_t intst) {
  switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {
    case (USBFS_UIS_TOKEN_IN | DEF_UEP0):
      if (setup_req_len == 0) {
        USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
      }
      if ((setup_req_type & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) {
        switch (setup_req_code) {
          case USB_GET_DESCRIPTOR: {
            uint16_t len = (setup_req_len >= EP0_SIZE) ? EP0_SIZE : setup_req_len;
            memcpy(ep0_buf, desc_ptr, len);
            setup_req_len -= len;
            desc_ptr += len;
            USBFSD->UEP0_TX_LEN = len;
            USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
            break;
          }
          case USB_SET_ADDRESS:
            USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | dev_addr;
            break;
          default:
            break;
        }
      }
      break;

    case (USBFS_UIS_TOKEN_IN | DEF_UEP1):
      USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
      USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      ep_busy[EP_NOTIFY]   = 0;
      break;

    case (USBFS_UIS_TOKEN_IN | DEF_UEP3):
      // The toggle is hardware handled (AUTO_TOG); only withdraw the data.
      USBFSD->UEP3_TX_CTRL = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
      ep_busy[EP_DATA_IN]  = 0;
      break;

    default:
      break;
  }
}

static void usbfs_out_stage(uint8_t intst) {
  switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {
    case (USBFS_UIS_TOKEN_OUT | DEF_UEP0):
      if (intst & USBFS_UIS_TOG_OK) {
        if ((setup_req_type & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
          setup_req_len = 0;
          if (setup_req_code == CDC_SET_LINE_CODING) {
            memcpy(line_coding, ep0_buf, sizeof(line_coding));
          }
          USBFSD->UEP0_TX_LEN  = 0;
          USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
        }
      }
      break;

    case (USBFS_UIS_TOKEN_OUT | DEF_UEP2): {
      USBFSD->UEP2_RX_CTRL ^= USBFS_UEP_R_TOG;
      const uint16_t len = USBFSD->RX_LEN;
      for (uint16_t i = 0; i < len; i++) {
        const uint16_t next = (rx_head + 1) & RX_MASK;
        if (next == rx_tail) {
          break; // ring full: drop the rest, the host will re-send
        }
        rx_ring[rx_head] = ep2_buf[i];
        rx_head          = next;
      }
      if (ring_count(rx_head, rx_tail, RX_MASK) > (USBFS_CDC_RX_RING_SIZE - EP2_SIZE)) {
        // Almost full: throttle until the application drains it.
        rx_naked             = true;
        USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
      }
      break;
    }

    default:
      break;
  }
}

// Called from USBHD_IRQHandler (== USBFS_IRQHandler) in hw/bsp/ch32v20x/family.c.
// That ISR is shared with TinyUSB's host/USBFS role, so this example takes it
// over through the weak usbfs_isr_hook() instead of defining the vector again.
void usbfs_isr_hook(void) {
  const uint8_t intflag = USBFSD->INT_FG;
  const uint8_t intst   = USBFSD->INT_ST;

  if (intflag & USBFS_UIF_TRANSFER) {
    switch (intst & USBFS_UIS_TOKEN_MASK) {
      case USBFS_UIS_TOKEN_IN:
        usbfs_in_stage(intst);
        break;
      case USBFS_UIS_TOKEN_OUT:
        usbfs_out_stage(intst);
        break;
      case USBFS_UIS_TOKEN_SETUP:
        usbfs_setup_stage();
        break;
      default:
        break;
    }
    USBFSD->INT_FG = USBFS_UIF_TRANSFER;
  } else if (intflag & USBFS_UIF_BUS_RST) {
    usbfs_device_reset_state();
    USBFSD->DEV_ADDR = 0;
    usbfs_ep_init();
    USBFSD->INT_FG = USBFS_UIF_BUS_RST;
  } else if (intflag & USBFS_UIF_SUSPEND) {
    usbfs_delay_us(10);
    if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) {
      dev_sleep |= 0x02;
    } else {
      dev_sleep &= (uint8_t)~0x02;
    }
    USBFSD->INT_FG = USBFS_UIF_SUSPEND;
  } else {
    USBFSD->INT_FG = intflag;
  }
}
