# Dual: HID device + CDC device (input bridge)

Two USB **device** controllers on one chip:

```
 controlling PC ──USB1──► FSDev (port 0) ──► TinyUSB device stack ──► HID: keyboard + pointers ──► target host
                ──USB2──► USBFS (port 1) ──► USBFS DCD + mini CDC-ACM ──► frames ──► drives the HID side above
```

The PC writes 12-byte frames to the CDC port; the firmware turns them into HID
reports on the other port, so the target machine sees a keyboard, an absolute
and a relative pointer, and a system-control input device.

## Why two stacks

TinyUSB's device stack is single-instance (`src/device/usbd.c` keeps one
`_usbd_rhport` and one set of class-driver state), so it cannot serve two device
controllers at once. This example therefore runs:

- **TinyUSB's device stack** on port 0 (FSDev) for HID, and
- a **mini CDC-ACM stack** (`src/usbfs_cdc.c`: descriptors, standard and class
  requests, two bulk endpoints, ring buffers) on port 1 (USBFS), on top of a
  copy of TinyUSB's own USBFS controller driver (`src/usbfs_dcd.c`, from
  `src/portable/wch/dcd_ch32_usbfs.c`, symbols prefixed `usbfs_dcd_`).

Only the enumeration and the CDC class on port 1 are new: the controller
handling in `usbfs_dcd.c` is the upstream file, so port 1 is proven code down
to the register accesses. The first version used a trimmed WCH `SimulateCDC`
driver instead and never transmitted on this board: the host polled the data IN
endpoint and got NAKs forever (usbmon: thousands of NAKs, zero transfers), even
with a transfer armed by hand in the debugger, while TinyUSB's driver transmits
on the same board, port and host.

The BSP owns the USBFS interrupt vector (`USBHD_IRQHandler`), so both roles
share it through the weak `usbfs_isr_hook()` exported by
`hw/bsp/ch32v20x/family.c` — empty for every other build, overridden here. The
DCD pushes events into a small queue that `usbfs_cdc_task()` drains, so no USB
work happens in the interrupt beyond the register handling; `src/usbfs_mini.h`
is the interface between the two.

## Wire protocol

Little-endian, fixed length (Chinese protocol reference:
[PROTOCOL.zh-CN.md](PROTOCOL.zh-CN.md)):

| offset | size | field |
|---|---|---|
| 0 | 2 | header: `0x8257` keyboard, `0x8357` absolute mouse, **`0x8557` relative mouse**, **`0x8657` system key** |
| 2 | 8 | payload |
| 10 | 2 | tail: `0x0900` |

On the wire the header/tail bytes are `57 82` / `57 83` / `57 85` / `57 86`
and `00 09`. The first two are the previous ESP32 product's, which also had a
button-only mouse frame (`0x8457`, `57 84`): that one is **removed**, because
every mouse frame already carries the button bitmap and a click needs no frame
of its own (see **Mouse payload**). Host software that still sends `0x8457`
keeps working, only its clicks are dropped — the parser rejects the header as
noise.

Device → host frames use `0x8757` (status), and the host polls with `0x8758`
(query); both are described under **Status frame** below.

### Keyboard payload (`data[0..7]`)

| offset | type | meaning |
|---|---|---|
| 0 | u8 | modifier (bit0 LCtrl … bit7 RGUI) |
| 1 | u8 | reserved, 0 |
| 2…7 | u8[6] | HID keycodes, 0 = unused slot |

### Mouse payload (`data[0..7]`)

| offset | type | meaning |
|---|---|---|
| 0 | u8 | buttons: bit0 left, bit1 right, bit2 middle (the HID descriptor declares 5 keys; only these three are defined) |
| 1 | u8 | padding, 0 |
| 2…3 | i16 | X: absolute 0…4096 (`0x8357`) or delta (`0x8557`) |
| 4…5 | i16 | Y: same convention as X |
| **6** | **i8** | **wheel (vertical), relative delta per frame** |
| **7** | **i8** | **pan (horizontal), relative delta per frame** |

Rules applied by the firmware:

- **Position**: `x`/`y` (0…4096) map to the 0…32767 absolute range of the HID
  report descriptor (`×8`, clamped).
- **Clicks**: there is no button-only frame, because the button bitmap rides
  every mouse frame. Two ways to click, both as a press frame plus a release
  frame:
  - `0x8557` with `dx`/`dy` = 0 and only the buttons changed needs **no
    position**; the cursor stays where the target OS has it. Use this when the
    host does not track the absolute pointer.
  - `0x8357` repeating the position of the last absolute frame also clicks in
    place, but the host has to track that position: `x`/`y` both 0 mean the
    corner, not "wherever the pointer was".
  - Press and release must go to the **same** pointer: the absolute and the
    relative device keep separate button state.
- **Scroll-only frames**: a `0x8357` frame whose `x`/`y` are both 0 but which
  carries wheel/pan keeps the last position — otherwise the pointer would jump
  to the corner on every scroll.
- **Wheel/pan** are deltas. Several frames can arrive before one HID report
  goes out (1 ms interval), so they are accumulated and saturated to ±127.
  They are cleared once a report is actually sent.
- The old ESP32 firmware carried these two bytes on the wire but dropped them
  (its report descriptor had no wheel usage), so the wheel never worked there.

### System key payload (`data[0..7]`, header `0x8657`)

| offset | type | meaning |
|---|---|---|
| 0 | u8 | `0` release, `1` power down, `2` sleep, `3` wake up |
| 1…7 | u8 | padding, 0 |

System keys are momentary: the firmware sends the press report and
automatically sends the release report on the following millisecond, so a host
can just send `code` and does not need to track button-up.

### Relative pointer (`0x8557`)

`x`/`y` are deltas in device units, not a position: the target OS moves its own
cursor. Deltas are accumulated across frames and split into one report per
millisecond (max ±127 per axis per report), so no movement is lost when the
host sends faster than the 1 kHz endpoint.

Zero deltas with only the button bitmap changed is a plain click: the frame
still produces one report, so the button press (and its release) reach the
target without any coordinate at all.

The relative pointer lives on a **second HID interface**. Linux's hid-input
ignores a HID usage that is already mapped on the same device, and the absolute
collection already claims `Wheel(0x38)`; keeping both on one interface silently
lost relative scrolling. Two interfaces also keep the system-control keys on
their own input device.

Frames are re-synchronised by a byte-stream parser: it scans for a valid header
and validates the tail, dropping one byte and retrying on mismatch, so lost or
garbage bytes cost at most one frame.

### Status frame (`0x8757`, device → host)

The reverse direction carries status only. A 12-byte `0x8757` frame is sent when
something changes (keyboard LED state, the host power state, DTR/RTS) and when
the host asks with `0x8758`, whose `token` byte is echoed back. Opening or
closing the port counts as a change too (DTR edge), and so does the port
appearing for the first time, so a fresh reader always gets one frame at once.

There is **no heartbeat**: nothing is sent while nothing happens, so a quiet
link is normal and `uptime` is a timestamp of the last event rather than a
liveness counter. A host that wants the current state sends the query
(measured round trip: ~0.1 ms). Measured on the board: opening the port yields
exactly one frame, 7 s of idle yields none, each DTR/RTS edge and each query
yields one, and five queries sent while RTS was low were all answered.

| offset | size | field |
|---|---|---|
| 0 | 2 | header `0x8757` (`57 87`) |
| 2 | 1 | state: 0 unmounted, 1 active, 2 suspended |
| 3 | 1 | HID interfaces configured (0 or 2) |
| 4 | 2 | uptime seconds at the moment the frame was built (wraps ~18 h) |
| 6 | 1 | flags: bit0 = DTR, bit1 = RTS, as set by the host |
| 7 | 1 | keyboard LEDs: bit0 num, bit1 caps, bit2 scroll, bit3 compose, bit4 kana |
| 8 | 1 | token, echoed from the query |
| 9 | 1 | reserved |
| 10 | 2 | tail `0x0900` (`00 09`) |

`flags` bit1 is new: it reports the host's own RTS line, which the firmware
records but never uses for gating (a host that leaves DTR/RTS low still
receives everything). Older host software that reads only bit0 is unaffected.
CTS is **not** reported: `SET_CONTROL_LINE_STATE` has no bit for the device to
read it from, the `SERIAL_STATE` notification has none either, and Linux's
`cdc_acm` hardcodes `TIOCM_CTS` on — a CTS bit would be fiction.

`leds` is the host's own keyboard LED output report (the 5-bit report the
keyboard collection declares): `tud_hid_set_report_cb()` stores it,
`tud_hid_get_report_cb()` answers a host `GET_REPORT` from the stored value, and
a changed bitmap becomes a status event. The first report arrives right after
mount (num lock on), and each toggle logs a line:

```
[8011] hid: leds num=1 caps=1 scroll=0
[9758] hid: leds num=1 caps=0 scroll=0
```

### Status query (`0x8758`, host → device)

| offset | type | meaning |
|---|---|---|
| 0 | 2 | header `0x8758` (`58 87`) |
| 2 | 1 | token, echoed in the `0x8757` reply |
| 3…9 | u8 | padding, 0 |
| 10 | 2 | tail `0x0900` (`00 09`) |

Because nothing is sent while nothing happens, a host that wants the current
state asks for it: `tools/cdc_tui.py` (`t`) times the round trip, and
`tools/bridge_check.py` probes Num Lock state this way, retrying the toggle
because a host sometimes coalesces two quick presses into one report.

## Build

```bash
cd examples/dual/hid_cdc_bridge
make BOARD=nanoch32v203 all
# or: cmake -B build -DBOARD=nanoch32v203 -G Ninja . && cmake --build build
```

```bash
make BOARD=nanoch32v203 flash      # wlink; see ../../../nanoch32v203.md for openocd
```

This example is CH32V20x only (`only.txt`: `mcu:CH32V20X`), because it needs the
two device IPs of that family (FSDev and USBFS); `nanoch32v203` is the board it
is developed and measured on.

> **Clean build after a config change**: the make path tracks no header
> dependencies, and a stale object built with `CFG_TUD_HID=1` makes `hidd_open`
> assert when the second HID interface is registered. Remove the build
> directory after editing `src/tusb_config.h`.

## Use

1. Plug **USB1** into the controlling PC (also the one running the script).
2. Plug **USB2** into the target machine.
3. On the controlling PC, the CDC port shows up as `/dev/ttyACM*`
   (VID:PID `cafe:4001`, "TinyUSB CDC Bridge", serial `203C10002`); the HID side
   of *that same PC* is `cafe:4004` ("TinyUSB HID Bridge", serial `203C10001`) —
   distinct product strings and serials so `lsusb` / device manager can tell the
   two apart (that is only for testing — in production the HID cable goes to the
   target).
4. Send frames, e.g. with `tools/bridge_check.py`:

```bash
./tools/bridge_check.py /dev/ttyACM2
```

Both pointers are named "<product> Mouse", so the test tells them apart by
capability: the absolute one is the device with `ABS` axes. The system-key check
grabs its input device (`EVIOCGRAB`), so a test run cannot power the host off.

### Interactive TUI

```bash
./tools/cdc_tui.py            # auto-detects the cafe:4001 CDC port
./tools/cdc_tui.py --selftest # no UI: inject a few frames, then query the state
```

`--selftest` follows each injected frame with a `0x8758` status query, since
status frames are event driven and a bare "nothing arrived" would not prove the
IN channel still works.

Two modes, switched with a single key (nothing is typed on a command line):

| mode | what a key does | how to get there |
|---|---|---|
| `COMMAND` | the key is a hotkey: arrows move the relative pointer, `[` / `]` left / right click on the relative pointer (press, then release 50 ms later — a terminal cannot report a key release), `m` absolute centre, `w`/`s` wheel, `r` eight steps back to the top-left corner, `c` Caps Lock, `p`/`l`/`k` power / sleep / wake, `t` status query, `i` raw hex of the last status frame, `:hex` / `:abs` / `:rel` raw commands, `q` quit | startup default, `Ctrl-]` |
| `TYPING` | the key is forwarded to the machine on the HID port: letters, capitals (shift is added), digits, punctuation, Enter / Tab / Space / Backspace, arrow keys. Hotkeys are off here, so `m`, `w`, `q` are typed instead of commanding | `Ctrl-]`; leave with `Ctrl-]` or `Esc` |

The current mode is in the title bar, and the key list on the right always
describes the mode you are in. Every status frame that comes back is decoded in
the log pane (state, interface count, uptime, DTR/RTS, LEDs). The toggle is
`Ctrl-]` (`0x1d`, one byte) rather than a function key because IDEs such as VS
Code grab `F1` before the terminal sees it; `F1` still works where the terminal
forwards it. `Esc` only leaves `TYPING`; in `COMMAND` it (like `q`) quits.

> **Power note**: the two Type-C ports are wired straight to the chip. When the
> two cables go to **different** machines, their 5 V VBUS rails meet. Use a
> VBUS-less cable on the HID side or an isolator. Testing both ports on the
> same PC is safe.

## Layout

| file | role |
|---|---|
| `src/main.c` | bridge policy: CDC → HID, report pacing, LED, status frames, USART1 log |
| `src/bridge_protocol.c` | 12-byte frame parser / re-synchronisation |
| `src/usb_descriptors.c` | HID: keyboard + absolute mouse (interface 0), relative mouse + system control (interface 1) |
| `src/usbfs_cdc.c` | mini CDC-ACM on the second controller: descriptors, requests, ring buffers, serialized TX pump |
| `src/usbfs_dcd.c` | TinyUSB's WCH USBFS controller driver, copied and renamed `usbfs_dcd_*` |
| `src/usbfs_mini.h` | interface between the DCD copy and the mini stack (events in, endpoint calls out) |
| `tools/bridge_check.py` | host-side acceptance test (keyboard, both pointers, clicks, wheel, system key, LED read-back, resync; Linux, needs `input` group; `--no-evdev` only sends frames) |
| `tools/cdc_tui.py` | interactive TUI (COMMAND / TYPING modes) and `--selftest` |
| `PROTOCOL.zh-CN.md` | Chinese protocol reference for the CDC link |

Everything runs from one loop in `main()`, in this order:

```c
tud_task();       // HID stack on port 0
usbfs_cdc_task(); // port 1: enumeration events, then the serialized TX pump
cdc_rx_task();    // CDC bytes -> frames -> HID state
hid_tx_task();    // at most one pending report per pass, remote wakeup if suspended
status_task();    // HID-side state -> one status frame, only when it changed
log_task();       // USART1 diagnostics
led_blinking_task();
```

## Host suspend vs unplug

The CH32V203 FSDev controller has no VBUS sensing, and its DCD says so
(`src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c`: suspend is asserted for both
suspend and unplug, so only suspend is reported). Consequence, measured with a
host that goes to sleep:

```
_usbd_dev.suspended = 1, cfg_num = 1, remote_wakeup_en = 1
blink_interval_ms   = 2500          # tud_suspend_cb(), not tud_umount_cb()
```

- A sleeping host gives `tud_suspend_cb()`; `tud_mounted()` stays true and
  `tud_umount_cb()` is never called.
- Unplugging the cable or powering the host off gives the same suspend event,
  not an unmount, so the application cannot tell them apart either.
- Because the host enabled remote wakeup, any frame injected while it sleeps
  raises `tud_remote_wakeup()` and pulls the bus out of suspend.

Getting a real unmount would mean driving the HID role from the USBFS
controller (`USBFS_SR_VBUS_VLD`) instead of FSDev, or adding an SOF-timeout
check in the application.

## Host-side serial settings

The CDC link carries binary frames, so the host tool must take the port out of
**canonical mode** (raw mode). In canonical mode `read()` only returns at a
line delimiter, which a binary frame almost never contains, so the tty appears
to stall: `cat /dev/ttyACM1` shows a burst and then goes quiet, while `minicom`
works because it sets raw mode itself. Disabling flow control alone does not
help; `-icanon` is what matters.

```python
tty.setraw(fd)                                   # no ICANON / ICRNL / OPOST
attrs = termios.tcgetattr(fd)
attrs[2] &= ~termios.CRTSCTS
attrs[0] &= ~(termios.IXON | termios.IXOFF)
termios.tcsetattr(fd, termios.TCSANOW, attrs)
```

**This was misdiagnosed as a firmware bug for a while**: the failure looked
exactly like "the device stops transmitting after a few seconds", and several
pump rewrites were spent on it before the termios setting was found. Bisecting
the branch later showed every firmware version works with a raw-mode reader.

`tools/cdc_tui.py` and `tools/bridge_check.py` already do this.

### Device-side note: a status frame is a short packet

The CDC data IN endpoint has a 64-byte max packet and a status frame is 12
bytes, so the controller only reports the transfer as complete when the host
issues one more IN token. The pump therefore stays serialized on that
completion (it never re-arms over a live transfer), and a 1 s safety valve
retires a frame whose host has stopped reading, so the endpoint cannot stay
pending forever. A host that reads in raw mode completes a frame in
milliseconds and never trips the valve.

## Limits

- Keyboard is 6-key rollover (`TUD_HID_REPORT_DESC_KEYBOARD`), no NKRO.
- Both pointer flavours are exposed at once (absolute report ID 2, relative
  report ID 3); the host picks. There is no mode switch to negotiate.
- Only the three system control keys (power down / sleep / wake up) are
  mapped; consumer/media keys would be another report ID.
- The CDC stack is a mini stack: hand-written descriptors and request handling,
  a notification endpoint that is opened and answered but never queued, and
  suspend/resume that is recorded, not acted on.
- Port 1 uses EP2 for both directions (OUT at +0, IN at +64): EP3's IN path is
  unreliable on this part, and the DCD already handles a bidirectional EP2.
- Frames are applied as they arrive; mouse moves are coalesced to one pending
  report so a burst cannot outrun the 1 ms interrupt endpoint.
- A click is two frames (press, release) of either mouse frame and needs no
  button-only frame; the relative one needs no position at all.

## Diagnostics over USART1

USART1 at 115200 is the only log channel that survives a host suspend, so it
carries what the USB stacks see while the control link is quiet:

```
[123456] hid: mounted
[123456] cdc: host opened port (DTR)
[125000] hid: bus suspend
[125004] hid: pending report -> remote wakeup
[125030] hid: bus resume
[125030] hid: system key wake up
[128456] stats: frames=42 reports=40 bad=0 hid=mounted cdc=open caps=0
[128456] cdc-tx: arms=3 done=3 stuck=0 timeout=0 isr_other=0
```

State transitions (`hid:`/`cdc:`) are logged once per change: mount/unmount,
bus suspend/resume, CDC port open/close, every system key press, and the first
pending report while suspended — that last line is the remote-wakeup trigger.
`stats` is printed every 5 s and includes `caps=`, the Caps Lock LED state.
`bad` counts parser re-syncs, and the `cdc-tx` line is the health of the IN
pump: `done` should track `arms`, `timeout` counts safety-valve resets (a host
that stopped reading) and `stuck` a failed arming.

Measured with a sleeping host: `hid: bus suspend` appears 1.3 s after the host
sleeps and no unmount follows, then a wake frame produces the pending-report
line and `hid: bus resume` 4 ms later, followed by the system key itself.

## Measured (nanoch32v203, xpack GCC15.2, both ports on one Linux host)

| check | result |
|---|---|
| build (make / cmake) | 21368 B flash (9.3% of 224 K), 7184 B RAM (35.1% of 20 K) |
| enumeration | `cafe:4004` **TinyUSB HID Bridge** and `cafe:4001` **TinyUSB CDC Bridge**; the HID side yields 4 input devices: Keyboard, Mouse (ABS 0…32767), Mouse (REL), System Control |
| keyboard frame | `KEY_A` down/up seen on `/dev/input/eventX` |
| absolute position | `2048/4096` → `16384/32767`, `1024/4096` → `8192/32767` |
| click, absolute (two `0x8357` frames repeating the position) | `BTN_LEFT` down, pointer does not move |
| click, relative (two `0x8557` frames, zero deltas) | `BTN_LEFT` down/up on the relative device, no position state |
| wheel / pan | `REL_WHEEL` +1 / −1, `REL_HWHEEL` +1, pointer does not move (scroll-only) |
| wheel + position in one frame | `REL_WHEEL` 2 **and** `ABS_X/Y` updated |
| relative pointer (`0x8557`) | `REL_X`/`REL_Y` deltas and `REL_WHEEL` on the relative input device |
| relative pointer, visual | on the target PC a 100x100 unit square (10 steps of 10 units per side, 100 ms apart) traced twice and closed back to the start, no drift and no pointer jump |
| system key (`0x8657`, power down) | `KEY_POWER` press **and** automatic release on the System Control device |
| cross-machine run | HID port on a second PC, CDC on the first: one `0x8657 code=1` frame put the target to sleep/shut down |
| cross-machine sleep + wake | `code=2` suspended the target, `code=3` woke it up — confirms the remote-wakeup + `KEY_WAKEUP` path on real hardware |
| suspend timing | `hid: bus suspend` logged 1.3 s after the host sleeps and no unmount follows; a wake frame gives the pending-report line, then `hid: bus resume` 4 ms later |
| status channel | opening the port yields exactly one frame, 7 s idle yields none, each DTR/RTS edge and each query yields one, the token is echoed, and 5/5 queries were answered with RTS low |
| keyboard LEDs | the first LED output report arrives right after mount (num lock on); a Caps Lock toggle logs `hid: leds num=1 caps=1 scroll=0` then `caps=0` and pushes a status frame |
| same, on current `origin/master` | rebuilt on top of upstream master (only the `usbfs_isr_hook()`/SystemView conflict in family.c), make + cmake pass, the check suite (then 23 checks) and sleep + wake re-verified cross-machine |
| frame split across writes | reassembled correctly |
| 3 frames in one write | all applied, last one wins |
| 500-frame burst (6012 B, blocking write) | no loss, final position exact |
| garbage before a valid frame | re-synchronised within one frame |

The size figures are from the build that made status frames event driven;
the commits after it only touched the host tools, so they still describe the
current firmware. The behavioural rows were measured as each feature landed, on
the two-port rig (both ports on one Linux host unless stated otherwise).
Re-measure sizes with `tools/code_size.py` and re-run `tools/bridge_check.py`
after changing the firmware. The click rows need a run with the two-frame
forms, since the button-only frame they were measured with is gone.
