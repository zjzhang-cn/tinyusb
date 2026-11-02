/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

// This example drives two USB device controllers at the same time:
//   - HID keyboard + absolute mouse on port 0 (FSDev, via TinyUSB)
//   - CDC-ACM serial on port 1 (USBFS, via the WCH device driver)
// Frames received on the CDC port are forwarded to the HID port, so a host
// attached to the CDC side can inject keyboard/mouse input into a host
// attached to the HID side. See README.md for the frame format.

#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "bridge_protocol.h"
#include "usb_descriptors.h"
#include "usbfs_cdc.h"

//--------------------------------------------------------------------+
// HID report state
//--------------------------------------------------------------------+

// Absolute mouse position is 0..32767 in the report descriptor, while the
// wire protocol carries 0..4096 (matching the previous ESP32 based product).
#define MOUSE_POS_IN_MAX  4096
#define MOUSE_POS_OUT_MAX 32767

typedef struct {
  bool    dirty;
  uint8_t buttons;
  int16_t x;
  int16_t y;
  int8_t  wheel;
  int8_t  pan;
} mouse_state_t;

static mouse_state_t mouse_state;
static struct {
  bool    dirty;
  uint8_t buf[8]; // modifier, reserved, keycode[6]
} kbd_state;

static uint32_t        blink_interval_ms = 250;
static bridge_parser_t parser;

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+

static int16_t scale_abs_position(int16_t in) {
  if (in <= 0) {
    return 0;
  }
  if (in >= MOUSE_POS_IN_MAX) {
    return MOUSE_POS_OUT_MAX;
  }
  // 4096 logical units map onto the 0..32767 range of the report descriptor.
  return (int16_t)(in * 8);
}

// Wheel/pan are relative deltas per frame; several frames may be coalesced into
// one report, so accumulate (saturating) instead of overwriting.
static int8_t saturating_add_i8(int8_t sum, int8_t delta) {
  const int16_t total = (int16_t)sum + delta;
  if (total > 127) {
    return 127;
  }
  if (total < -127) {
    return -127;
  }
  return (int8_t)total;
}

static void apply_keyboard(const bridge_keyboard_t *kbd) {
  kbd_state.buf[0] = kbd->modifier;
  kbd_state.buf[1] = kbd->reserved;
  memcpy(&kbd_state.buf[2], kbd->keycode, sizeof(kbd->keycode));
  kbd_state.dirty = true;
}

static void apply_mouse_move(const bridge_mouse_t *mouse) {
  const bool has_position = (mouse->x != 0) || (mouse->y != 0);
  const bool has_scroll   = (mouse->wheel != 0) || (mouse->pan != 0);

  mouse_state.buttons = mouse->buttons;

  // A scroll-only frame may carry 0,0 as its position; applying it would
  // teleport the pointer to the corner of an absolute device, so keep the
  // previous position unless the frame actually carries one.
  if (has_position || !has_scroll) {
    mouse_state.x = scale_abs_position(mouse->x);
    mouse_state.y = scale_abs_position(mouse->y);
  }

  mouse_state.wheel = saturating_add_i8(mouse_state.wheel, mouse->wheel);
  mouse_state.pan   = saturating_add_i8(mouse_state.pan, mouse->pan);
  mouse_state.dirty = true;
}

static void apply_mouse_button(const bridge_mouse_t *mouse) {
  // Button-only frame: keep the last absolute position (sending 0,0 would
  // teleport the pointer to the corner on an absolute mouse). Pending wheel
  // deltas are kept until a report actually goes out.
  mouse_state.buttons = mouse->buttons;
  mouse_state.dirty   = true;
}

static void handle_event(const bridge_evt_t *evt) {
  switch (evt->type) {
    case BRIDGE_EVT_KEYBOARD:
      apply_keyboard(&evt->keyboard);
      break;
    case BRIDGE_EVT_MOUSE_MOVE:
      apply_mouse_move(&evt->mouse);
      break;
    case BRIDGE_EVT_MOUSE_BUTTON:
      apply_mouse_button(&evt->mouse);
      break;
    default:
      break;
  }
}

//--------------------------------------------------------------------+
// Tasks
//--------------------------------------------------------------------+

// Drain the CDC receive ring and turn frames into HID reports.
static void cdc_rx_task(void) {
  uint8_t  buf[64];
  uint32_t count;

  while ((count = usbfs_cdc_read(buf, sizeof(buf))) > 0) {
    for (uint32_t i = 0; i < count; i++) {
      bridge_evt_t evt;
      if (bridge_parser_feed(&parser, buf[i], &evt)) {
        handle_event(&evt);
      }
    }
  }
}

// Send at most one pending report per call; keyboard first since key state is
// more important than pointer position.
static void hid_tx_task(void) {
  if (!kbd_state.dirty && !mouse_state.dirty) {
    return;
  }

  if (tud_suspended()) {
    tud_remote_wakeup();
    return;
  }

  if (!tud_hid_ready()) {
    return;
  }

  if (kbd_state.dirty) {
    if (tud_hid_n_report(0, REPORT_ID_KEYBOARD, kbd_state.buf, sizeof(kbd_state.buf))) {
      kbd_state.dirty = false;
    }
    return;
  }

  // buttons, X (2), Y (2), wheel, pan - the report ID is prepended by TinyUSB
  uint8_t report[7];
  report[0] = mouse_state.buttons;
  report[1] = (uint8_t)(mouse_state.x & 0xff);
  report[2] = (uint8_t)((mouse_state.x >> 8) & 0xff);
  report[3] = (uint8_t)(mouse_state.y & 0xff);
  report[4] = (uint8_t)((mouse_state.y >> 8) & 0xff);
  report[5] = (uint8_t)mouse_state.wheel;
  report[6] = (uint8_t)mouse_state.pan;

  if (tud_hid_n_report(0, REPORT_ID_MOUSE, report, sizeof(report))) {
    mouse_state.dirty = false;
    mouse_state.wheel = 0;
    mouse_state.pan   = 0;
  }
}

static void led_blinking_task(void) {
  static uint32_t start_ms  = 0;
  static bool     led_state = false;

  if (tusb_time_millis_api() - start_ms < blink_interval_ms) {
    return;
  }
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state;
}

//--------------------------------------------------------------------+
// Main
//--------------------------------------------------------------------+

int main(void) {
  board_init();

  printf("TinyUSB HID CDC bridge\r\n");
  printf("  port 0 (USB1): HID keyboard + absolute mouse\r\n");
  printf("  port 1 (USB2): CDC-ACM control channel\r\n");

  tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  usbfs_cdc_init();

  board_init_after_tusb();
  bridge_parser_init(&parser);

  while (1) {
    tud_task();       // HID device stack (port 0)
    usbfs_cdc_task(); // CDC transmit pump (port 1)
    cdc_rx_task();    // CDC -> HID
    hid_tx_task();    // pending HID reports
    led_blinking_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void) {
  blink_interval_ms = 1000;
}

void tud_umount_cb(void) {
  blink_interval_ms = 250;
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = 2500;
}

void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? 1000 : 250;
}

// Invoked when received SET_REPORT control request or data received on OUT endpoint
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, const uint8_t *buffer,
                           uint16_t bufsize) {
  (void)instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT) {
    // Keyboard LED report: keep the last state, nothing to forward (the CDC
    // channel is write-only towards the device).
    (void)report_id;
    (void)buffer;
    (void)bufsize;
  }
}

// Invoked when received GET_REPORT control request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}
