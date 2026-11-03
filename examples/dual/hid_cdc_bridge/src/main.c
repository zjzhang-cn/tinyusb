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

// Relative pointer: deltas accumulate so a burst of frames does not lose
// movement when only one report per millisecond can be sent.
static struct {
  bool    dirty;
  uint8_t buttons;
  int16_t dx;
  int16_t dy;
  int8_t  wheel;
  int8_t  pan;
} rel_state;

// System control keys (power down / sleep / wake up) are momentary: the press
// report is followed by a release report on the next tick.
static struct {
  bool    press_pending;
  bool    release_pending;
  uint8_t code;
} sys_state;

// Reported to the host over the CDC port: what the machine on the HID side is
// doing (enumerated / suspended / gone) plus a heartbeat.
static struct {
  bool     mounted;
  bool     suspended;
  bool     dirty;
  uint8_t  token;  // echoed from the host's status query
  uint32_t last_ms;
} status_state = {.dirty = true};

// Keyboard LED state reported by the host (output report): bit0 num lock,
// bit1 caps lock, bit2 scroll lock, bit3 compose, bit4 kana.
static struct {
  bool    known;
  uint8_t leds;
} kbd_led;

static uint32_t        blink_interval_ms = 250;
static bridge_parser_t parser;

//--------------------------------------------------------------------+
// Diagnostics (USART1 @115200, independent of the CDC control channel)
//--------------------------------------------------------------------+

#define LOG_PERIOD_MS 5000

static struct {
  uint32_t events;      // frames parsed from CDC
  uint32_t reports;     // HID reports actually queued
  uint32_t bad_frame;   // parser re-syncs
  bool mounted;
  bool suspended;
  bool dtr;
  bool wake_logged;
  uint32_t next_ms;
} log_state;

// Device -> host status frame: the LED bitmap, so the controlling PC can show
// Caps Lock state without a second channel.
static void cdc_send_status(void) {
  uint8_t frame[BRIDGE_FRAME_LEN];
  bridge_build_status(frame, kbd_led.leds);
  usbfs_cdc_write(frame, sizeof(frame));
}

static void log_event(const char* what) {
  printf("[%lu] %s\r\n", (unsigned long) tusb_time_millis_api(), what);
}

static void log_task(void) {
  const bool mounted = tud_mounted();
  const bool suspended = tud_suspended();
  const bool dtr = usbfs_cdc_connected();
  const uint32_t now = tusb_time_millis_api();

  if (mounted != log_state.mounted) {
    log_state.mounted = mounted;
    log_event(mounted ? "hid: mounted" : "hid: unmounted");
  }
  if (suspended != log_state.suspended) {
    log_state.suspended = suspended;
    log_state.wake_logged = false;
    log_event(suspended ? "hid: bus suspend" : "hid: bus resume");
  }
  if (dtr != log_state.dtr) {
    log_state.dtr = dtr;
    log_event(dtr ? "cdc: host opened port (DTR)" : "cdc: host closed port");
  }

  if ((int32_t) (now - log_state.next_ms) >= 0) {
    log_state.next_ms = now + LOG_PERIOD_MS;
    printf("[%lu] stats: frames=%lu reports=%lu bad=%lu hid=%s cdc=%s caps=%u\r\n",
           (unsigned long) now, (unsigned long) log_state.events,
           (unsigned long) log_state.reports, (unsigned long) log_state.bad_frame,
           suspended ? "suspended" : (mounted ? "mounted" : "detached"),
           dtr ? "open" : "closed",
           kbd_led.known ? ((kbd_led.leds >> 1) & 1u) : 0u);
    printf("[%lu] cdc-tx: arms=%lu done=%lu stuck=%lu timeout=%lu isr_other=%lu\r\n", (unsigned long) now,
           (unsigned long) usbfs_dbg_tx_arms, (unsigned long) usbfs_dbg_tx_done,
           (unsigned long) usbfs_dbg_tx_stuck, (unsigned long) usbfs_dbg_tx_timeout,
           (unsigned long) usbfs_dbg_isr_other);
    cdc_send_status();  // heartbeat so a reconnecting host learns the LED state
  }
}

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

static int16_t saturating_add_i16(int16_t sum, int32_t delta) {
  const int32_t total = (int32_t)sum + delta;
  if (total > 32767) {
    return 32767;
  }
  if (total < -32768) {
    return -32768;
  }
  return (int16_t)total;
}

static void apply_mouse_rel_move(const bridge_mouse_rel_t *mouse) {
  // x/y are deltas here; accumulate so coalescing into one report per
  // millisecond keeps the total movement.
  rel_state.buttons = mouse->buttons;
  rel_state.dx      = saturating_add_i16(rel_state.dx, mouse->x);
  rel_state.dy      = saturating_add_i16(rel_state.dy, mouse->y);
  rel_state.wheel   = saturating_add_i8(rel_state.wheel, mouse->wheel);
  rel_state.pan     = saturating_add_i8(rel_state.pan, mouse->pan);
  rel_state.dirty   = true;
}

static void apply_system_key(const bridge_system_key_t *key) {
  if (key->code == BRIDGE_SYS_KEY_RELEASE) {
    sys_state.press_pending   = false;
    sys_state.release_pending = true;
  } else if (key->code <= BRIDGE_SYS_KEY_WAKE_UP) {
    sys_state.code          = key->code;
    sys_state.press_pending = true;
  }
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
    case BRIDGE_EVT_MOUSE_REL_MOVE:
      apply_mouse_rel_move(&evt->mouse_rel);
      break;
    case BRIDGE_EVT_SYSTEM_KEY:
      apply_system_key(&evt->system);
      break;
    case BRIDGE_EVT_STATUS_REQ:
      // Host polled us: answer as soon as the loop comes round.
      status_state.token = evt->status_req.token;
      status_state.dirty = true;
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
        log_state.events++;
        handle_event(&evt);
      }
    }
  }
}

// Send at most one pending report per call; keyboard first since key state is
// more important than pointer position.
static void hid_ext_tx_task(void) {
  if (!sys_state.press_pending && !sys_state.release_pending && !rel_state.dirty) {
    return;
  }

  if (!tud_hid_n_ready(HID_INSTANCE_EXT)) {
    return;
  }

  // System keys are momentary: press first, release on the next tick.
  if (sys_state.press_pending) {
    const uint8_t report = sys_state.code;
    if (tud_hid_n_report(HID_INSTANCE_EXT, REPORT_ID_SYSTEM, &report, sizeof(report))) {
      log_state.reports++;
      log_event(sys_state.code == BRIDGE_SYS_KEY_POWER_DOWN ? "hid: system key power down"
                : sys_state.code == BRIDGE_SYS_KEY_SLEEP    ? "hid: system key sleep"
                                                            : "hid: system key wake up");
      sys_state.press_pending   = false;
      sys_state.release_pending = true;
    }
    return;
  }

  if (sys_state.release_pending) {
    const uint8_t report = 0;
    if (tud_hid_n_report(HID_INSTANCE_EXT, REPORT_ID_SYSTEM, &report, sizeof(report))) {
      log_state.reports++;
      sys_state.release_pending = false;
    }
    return;
  }

  // Relative pointer: one report carries at most +-127 per axis, the rest stays
  // accumulated for the following ticks.
  const int8_t dx = rel_state.dx > 127 ? 127 : (rel_state.dx < -127 ? -127 : (int8_t)rel_state.dx);
  const int8_t dy = rel_state.dy > 127 ? 127 : (rel_state.dy < -127 ? -127 : (int8_t)rel_state.dy);

  uint8_t report[5];
  report[0] = rel_state.buttons;
  report[1] = (uint8_t)dx;
  report[2] = (uint8_t)dy;
  report[3] = (uint8_t)rel_state.wheel;
  report[4] = (uint8_t)rel_state.pan;

  if (tud_hid_n_report(HID_INSTANCE_EXT, REPORT_ID_MOUSE_REL, report, sizeof(report))) {
    log_state.reports++;
    rel_state.dx -= dx;
    rel_state.dy -= dy;
    rel_state.wheel = 0;
    rel_state.pan   = 0;
    rel_state.dirty = (rel_state.dx != 0) || (rel_state.dy != 0);
  }
}

static void hid_main_tx_task(void) {
  if (!kbd_state.dirty && !mouse_state.dirty) {
    return;
  }

  if (!tud_hid_n_ready(HID_INSTANCE_MAIN)) {
    return;
  }

  if (kbd_state.dirty) {
    if (tud_hid_n_report(HID_INSTANCE_MAIN, REPORT_ID_KEYBOARD, kbd_state.buf, sizeof(kbd_state.buf))) {
      log_state.reports++;
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

  if (tud_hid_n_report(HID_INSTANCE_MAIN, REPORT_ID_MOUSE, report, sizeof(report))) {
    log_state.reports++;
    mouse_state.dirty = false;
    mouse_state.wheel = 0;
    mouse_state.pan   = 0;
  }
}

static void hid_tx_task(void) {
  const bool pending =
    sys_state.press_pending || sys_state.release_pending || kbd_state.dirty || mouse_state.dirty || rel_state.dirty;
  if (!pending) {
    return;
  }

  if (tud_suspended()) {
    if (!log_state.wake_logged) {
      log_state.wake_logged = true;
      log_event("hid: pending report -> remote wakeup");
    }
    tud_remote_wakeup();
    return;
  }

  hid_ext_tx_task();  // system keys and relative pointer first
  hid_main_tx_task(); // keyboard, then absolute pointer
}

// Push a status frame on every state change and once per second. Only when the
// host has the CDC port open, so nothing queues up while nobody listens.
static void status_task(void) {
  const uint32_t now = tusb_time_millis_api();
  if (!status_state.dirty && (now - status_state.last_ms) < 1000u) {
    return;
  }

  // The bridge does not depend on the host asserting DTR: the CDC link counts
  // as connected once the port is enumerated, so a status change or a query
  // still gets answered on hosts that never set DTR.
  if (!usbfs_cdc_enumerated()) {
    return;
  }

  bridge_status_t st;
  memset(&st, 0, sizeof(st));
  st.state = status_state.mounted ? (status_state.suspended ? BRIDGE_STATE_SUSPENDED : BRIDGE_STATE_ACTIVE)
                                  : BRIDGE_STATE_UNMOUNTED;
  // Both HID interfaces come from the same configuration, so they are either
  // both configured (device configured) or neither.
  st.hid_itf_count = status_state.mounted ? 2 : 0;
  st.uptime_s      = (uint16_t)(now / 1000u);
  st.flags         = usbfs_cdc_line_state();  // bit0: DTR, bit1: RTS
  st.leds          = kbd_led.known ? kbd_led.leds : 0;
  st.token         = status_state.token;

  uint8_t        frame[BRIDGE_FRAME_LEN];
  const uint16_t header = BRIDGE_HDR_STATUS;
  const uint16_t tail   = BRIDGE_FRAME_TAIL;
  memcpy(frame, &header, sizeof(header));
  memcpy(frame + 2, &st, sizeof(st));
  memcpy(frame + BRIDGE_FRAME_LEN - 2, &tail, sizeof(tail));

  if (usbfs_cdc_write(frame, sizeof(frame)) == sizeof(frame)) {
    status_state.dirty   = false;
    status_state.last_ms = now;
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
    status_task();    // HID port state -> CDC
    log_task();
    led_blinking_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void) {
  blink_interval_ms      = 1000;
  status_state.mounted   = true;
  status_state.suspended = false;
  status_state.dirty     = true;
}

void tud_umount_cb(void) {
  blink_interval_ms      = 250;
  status_state.mounted   = false;
  status_state.suspended = false;
  status_state.dirty     = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms      = 2500;
  status_state.suspended = true;
  status_state.dirty     = true;
}

void tud_resume_cb(void) {
  blink_interval_ms      = tud_mounted() ? 1000 : 250;
  status_state.suspended = false;
  status_state.dirty     = true;
}

// Invoked when received SET_REPORT control request or data received on OUT endpoint
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, const uint8_t *buffer,
                           uint16_t bufsize) {
  if (instance != HID_INSTANCE_MAIN || report_type != HID_REPORT_TYPE_OUTPUT) {
    return;
  }
  if (report_id != 0 && report_id != REPORT_ID_KEYBOARD) {
    return;
  }
  if (bufsize == 0) {
    return;
  }

  // The CDC channel is write-only towards the device, so the LED state is
  // only kept here and printed on the UART log.
  if (!kbd_led.known || kbd_led.leds != buffer[0]) {
    kbd_led.known = true;
    kbd_led.leds = buffer[0];
    printf("[%lu] hid: leds num=%u caps=%u scroll=%u\r\n", (unsigned long) tusb_time_millis_api(),
           (buffer[0] >> 0) & 1u, (buffer[0] >> 1) & 1u, (buffer[0] >> 2) & 1u);
    status_state.dirty = true;
  }
}

// Invoked when received GET_REPORT control request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  if (instance == HID_INSTANCE_MAIN && report_type == HID_REPORT_TYPE_OUTPUT && reqlen >= 1 &&
      (report_id == 0 || report_id == REPORT_ID_KEYBOARD)) {
    buffer[0] = kbd_led.leds;
    return 1;
  }
  return 0;
}
