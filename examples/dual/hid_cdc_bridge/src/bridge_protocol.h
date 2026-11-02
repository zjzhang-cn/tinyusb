/*
 * Wire protocol carried over the CDC serial port.
 *
 * Kept byte-compatible with the existing "esp32_hid" product so the PC side
 * needs no change: fixed 12-byte little-endian frames
 *
 *     uint16 header | uint8 data[8] | uint16 tail
 *
 *   header 0x8257 -> keyboard  (modifier, reserved, keycode[6])
 *   header 0x8357 -> mouse move (button, reserved, x, y, wheel, pan)
 *   header 0x8457 -> mouse button (button, reserved, x, y, wheel, pan)
 *   tail   0x0900
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

#define BRIDGE_FRAME_LEN        12
#define BRIDGE_FRAME_TAIL       0x0900

#define BRIDGE_HDR_KEYBOARD     0x8257
#define BRIDGE_HDR_MOUSE_MOVE   0x8357
#define BRIDGE_HDR_MOUSE_BUTTON 0x8457

typedef enum {
  BRIDGE_EVT_NONE = 0,
  BRIDGE_EVT_KEYBOARD,
  BRIDGE_EVT_MOUSE_MOVE,
  BRIDGE_EVT_MOUSE_BUTTON,
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

typedef struct {
  bridge_evt_type_t type;
  union {
    bridge_keyboard_t keyboard;
    bridge_mouse_t    mouse;
  };
} bridge_evt_t;

typedef struct {
  uint8_t buf[BRIDGE_FRAME_LEN];
  uint8_t len;
} bridge_parser_t;

void bridge_parser_init(bridge_parser_t *parser);

// Feed one byte. Returns true when *evt was filled with a complete frame.
bool bridge_parser_feed(bridge_parser_t *parser, uint8_t byte, bridge_evt_t *evt);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_PROTOCOL_H_ */
