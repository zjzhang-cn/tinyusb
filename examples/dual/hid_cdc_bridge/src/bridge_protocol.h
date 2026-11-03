/*
 * Wire protocol carried over the CDC serial port.
 *
 * Kept byte-compatible with the existing "esp32_hid" product so the PC side
 * needs no change: fixed 12-byte little-endian frames
 *
 *     uint16 header | uint8 data[8] | uint16 tail
 *
 *   header 0x8257 -> keyboard  (modifier, reserved, keycode[6])
 *   header 0x8357 -> mouse move, absolute (button, reserved, x, y, wheel, pan)
 *   header 0x8457 -> mouse button (button, reserved, ...)
 *   header 0x8557 -> mouse move, relative (button, reserved, dx, dy, wheel, pan)
 *   header 0x8657 -> system key (power down / sleep / wake up)
 *   header 0x8757 -> status, device -> host only (state of the HID port)
 *   tail   0x0900
 *
 * The first three headers are the ones the previous ESP32 based product used;
 * 0x8557 and 0x8657 are additions and can be ignored by older host software.
 *
 * The parser is a byte-stream state machine: CDC delivers arbitrary chunk
 * boundaries, so frames are re-synchronised by scanning for a valid header
 * and validating the tail. A bad frame drops one byte and retries, which
 * recovers within a frame even after a lost byte.
 */

#ifndef BRIDGE_PROTOCOL_H_
#define BRIDGE_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BRIDGE_FRAME_LEN          12
#define BRIDGE_FRAME_TAIL         0x0900

#define BRIDGE_HDR_KEYBOARD       0x8257
#define BRIDGE_HDR_MOUSE_MOVE     0x8357
#define BRIDGE_HDR_MOUSE_BUTTON   0x8457
#define BRIDGE_HDR_MOUSE_REL_MOVE 0x8557
#define BRIDGE_HDR_SYSTEM_KEY     0x8657

// Device -> host only. The parser never accepts it as an input header.
#define BRIDGE_HDR_STATUS         0x8757

// Host -> device: "report your state now". data[0] is a token the device echoes
// in the status frame, so the host can match a reply to its query and notice a
// dropped one.
#define BRIDGE_HDR_STATUS_REQ     0x8758

// System key codes, matching the HID system control usage array (1..3).
#define BRIDGE_SYS_KEY_RELEASE    0
#define BRIDGE_SYS_KEY_POWER_DOWN 1
#define BRIDGE_SYS_KEY_SLEEP      2
#define BRIDGE_SYS_KEY_WAKE_UP    3

typedef enum {
  BRIDGE_EVT_NONE = 0,
  BRIDGE_EVT_KEYBOARD,
  BRIDGE_EVT_MOUSE_MOVE,
  BRIDGE_EVT_MOUSE_BUTTON,
  BRIDGE_EVT_MOUSE_REL_MOVE,
  BRIDGE_EVT_SYSTEM_KEY,
  BRIDGE_EVT_STATUS_REQ,
} bridge_evt_type_t;

typedef struct {
  uint8_t modifier;
  uint8_t reserved;
  uint8_t keycode[6];
} bridge_keyboard_t;

typedef struct {
  uint8_t buttons;
  int8_t  reserved;
  int16_t x;
  int16_t y;
  int8_t  wheel;
  int8_t  pan;
} bridge_mouse_t;

// Same 8-byte payload; x/y carry deltas instead of absolute coordinates.
typedef bridge_mouse_t bridge_mouse_rel_t;

typedef struct {
  uint8_t code; // BRIDGE_SYS_KEY_*
  uint8_t reserved[7];
} bridge_system_key_t;

typedef struct {
  uint8_t token;  // echoed in the status frame
  uint8_t reserved[7];
} bridge_status_req_t;

// Device -> host status frame. Sent when something changes and when the host
// asks with BRIDGE_HDR_STATUS_REQ (whose token is echoed), never on a timer, so
// the host can tell whether the HID side is alive and what power state the
// machine on that port is in.
typedef enum {
  BRIDGE_STATE_UNMOUNTED = 0, // powered off, cable out, or not enumerated
  BRIDGE_STATE_ACTIVE = 1,    // enumerated and the bus is active
  BRIDGE_STATE_SUSPENDED = 2, // the target suspended the bus (S3)
} bridge_state_t;

typedef struct {
  uint8_t  state;         // BRIDGE_STATE_*
  uint8_t  hid_itf_count; // HID interfaces configured (0 or 2)
  uint16_t uptime_s;      // seconds since boot, wraps every ~18 h
  uint8_t  flags;         // bit0: DTR (host opened the port), bit1: RTS
  uint8_t  leds;          // keyboard LED bitmap (bit0 num, bit1 caps, bit2 scroll)
  uint8_t  token;         // echoed from BRIDGE_HDR_STATUS_REQ
  uint8_t  reserved;
} bridge_status_t;

typedef struct {
  bridge_evt_type_t type;
  union {
    bridge_keyboard_t   keyboard;
    bridge_mouse_t      mouse;
    bridge_mouse_rel_t  mouse_rel;
    bridge_system_key_t system;
    bridge_status_req_t status_req;
  };
} bridge_evt_t;

typedef struct {
  uint8_t buf[BRIDGE_FRAME_LEN];
  uint8_t len;
} bridge_parser_t;

// Build a device -> host status frame. data[0] carries the keyboard LED bitmap
// (bit0 num lock, bit1 caps lock, bit2 scroll lock, bit3 compose, bit4 kana);
// the remaining payload bytes are reserved and zero.
void bridge_build_status(uint8_t frame[BRIDGE_FRAME_LEN], uint8_t leds);

void bridge_parser_init(bridge_parser_t *parser);

// Feed one byte. Returns true when *evt was filled with a complete frame.
bool bridge_parser_feed(bridge_parser_t *parser, uint8_t byte, bridge_evt_t *evt);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_PROTOCOL_H_ */
