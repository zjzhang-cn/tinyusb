# Dual: HID device + CDC device (input bridge)

Two USB **device** controllers on one chip:

```
            ┌─────────────────────────── nanoCH32V203 ───────────────────────────┐
            │                                                                   │
 PC ──USB1──┼─► Port0 / FSDev ─► TinyUSB device stack ─► HID keyboard + mouse  │──USB2──► target host
 (control)  │                                                    ▲              │         (sees a keyboard
            │                                                    │              │          and an absolute
 PC ──USB2──┼─► Port1 / USBFS ─► WCH USBFS CDC driver ────────────┘              │          pointer)
 (control)  │                                                                   │
            └───────────────────────────────────────────────────────────────────┘
```

The PC writes 12-byte frames to the CDC port; the firmware turns them into HID
reports on the other port, so the target machine sees a keyboard and an
**absolute** pointing device.

## Why two stacks

TinyUSB's device stack is single-instance (`src/device/usbd.c` keeps one
`_usbd_rhport` and one set of class-driver state), so it cannot serve two device
controllers at once. This example therefore runs:

- **TinyUSB** on port 0 (FSDev) for HID, and
- the **WCH USBFS device driver** (`src/usbfs_cdc.c`, trimmed from
  `hw/mcu/wch/ch32v20x/EVT/EXAM/USB/USBFS/DEVICE/SimulateCDC`) on port 1 for
  CDC-ACM.

The BSP owns the USBFS interrupt vector, so the driver is reached through the
weak `usbfs_isr_hook()` exported by `hw/bsp/ch32v20x/family.c`.

## Wire protocol

Byte-compatible with the previous ESP32 based product, so the host software
does not change. Little-endian, fixed length:

| offset | size | field |
|---|---|---|
| 0 | 2 | header: `0x8257` keyboard, `0x8357` absolute mouse move, `0x8457` mouse button, **`0x8557` relative mouse move**, **`0x8657` system key** |
| 2 | 8 | payload |
| 10 | 2 | tail: `0x0900` |

On the wire the header/tail bytes are `57 82` / `57 83` / `57 84` / `57 85` / `57 86`
and `00 09`. The first three are the original ESP32 product's; the last two are
additions, so older host software keeps working unchanged.

### Keyboard payload (`data[0..7]`)

| offset | type | meaning |
|---|---|---|
| 0 | u8 | modifier (bit0 LCtrl … bit7 RGUI) |
| 1 | u8 | reserved, 0 |
| 2…7 | u8[6] | HID keycodes, 0 = unused slot |

### Mouse payload (`data[0..7]`)

| offset | type | meaning |
|---|---|---|
| 0 | u8 | buttons: bit0 left, bit1 middle, bit2 right |
| 1 | u8 | padding, 0 |
| 2…3 | i16 | X: absolute 0…4096 (`0x8357`) or delta (`0x8557`) |
| 4…5 | i16 | Y: same convention as X |
| **6** | **i8** | **wheel (vertical), relative delta per frame** |
| **7** | **i8** | **pan (horizontal), relative delta per frame** |

Rules applied by the firmware:

- **Position**: `x`/`y` (0…4096) map to the 0…32767 absolute range of the HID
  report descriptor (`×8`, clamped).
- **Position-free frames**: a `0x8457` frame (button only) keeps the last
  position. A `0x8357` frame whose `x`/`y` are both 0 but which carries
  wheel/pan is treated as scroll-only and also keeps the last position —
  otherwise the pointer would jump to the corner on every scroll.
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

The relative pointer lives on a **second HID interface**. Linux's hid-input
ignores a HID usage that is already mapped on the same device, and the absolute
collection already claims `Wheel(0x38)`; keeping both on one interface silently
lost relative scrolling. Two interfaces also keep the system-control keys on
their own input device.

Frames are re-synchronised by a byte-stream parser: it scans for a valid header
and validates the tail, dropping one byte and retrying on mismatch, so lost or
garbage bytes cost at most one frame.

The reverse direction is not used: the CDC port is write-only towards the
device (keyboard LED reports are parsed and discarded).

## Build

```bash
cd examples/dual/hid_cdc_bridge
make BOARD=nanoch32v203 all
# or: cmake -B build -DBOARD=nanoch32v203 -G Ninja . && cmake --build build
```

```bash
make BOARD=nanoch32v203 flash      # wlink; see ../../nanoch32v203.md for openocd
```

## Use

1. Plug **USB1** into the controlling PC (also the one running the script).
2. Plug **USB2** into the target machine.
3. On the controlling PC, the CDC port shows up as `/dev/ttyACM*`
   (VID:PID `cafe:4001`, "TinyUSB CDC Bridge"); the HID side of *that same PC* is
   `cafe:4004` ("TinyUSB HID Bridge")
   (that is only for testing — in production the HID cable goes to the target).
4. Send frames, e.g. with `tools/bridge_check.py`:

```bash
./tools/bridge_check.py /dev/ttyACM2
```

> **Power note**: the two Type-C ports are wired straight to the chip. When the
> two cables go to **different** machines, their 5 V VBUS rails meet. Use a
> VBUS-less cable on the HID side or an isolator. Testing both ports on the
> same PC is safe.

## Layout

| file | role |
|---|---|
| `src/main.c` | bridge policy: CDC → HID, report pacing, LED |
| `src/bridge_protocol.c` | 12-byte frame parser / re-synchronisation |
| `src/usb_descriptors.c` | HID: keyboard + absolute mouse (interface 0), relative mouse + system control (interface 1) |
| `src/usbfs_cdc.c` | CDC-ACM on the second controller, ring buffers |
| `tools/bridge_check.py` | host-side acceptance test (Linux, needs `input` group) |

## Limits

- Keyboard is 6-key rollover (`TUD_HID_REPORT_DESC_KEYBOARD`), no NKRO.
- Both pointer flavours are exposed at once (absolute report ID 2, relative
  report ID 3); the host picks. There is no mode switch to negotiate.
- Only the three system control keys (power down / sleep / wake up) are
  mapped; consumer/media keys would be another report ID.
- The CDC stack is a trimmed WCH driver: no notification endpoint usage, no
  suspend/resume handling beyond clearing the interrupt flag.
- Frames are applied as they arrive; mouse moves are coalesced to one pending
  report so a burst cannot outrun the 1 ms interrupt endpoint.

## Measured (nanoch32v203, xpack GCC15.2, both ports on one Linux host)

| check | result |
|---|---|
| build (make / cmake) | 16884 B flash (7.4% of 224 K), 4424 B RAM (21.6% of 20 K) |
| enumeration | `cafe:4004` **TinyUSB HID Bridge** and `cafe:4001` **TinyUSB CDC Bridge**; the HID side yields 4 input devices: Keyboard, Mouse (ABS 0…32767), Mouse (REL), System Control |
| keyboard frame | `KEY_A` down/up seen on `/dev/input/eventX` |
| absolute position | `2048/4096` → `16384/32767`, `1024/4096` → `8192/32767` |
| button-only frame | `BTN_LEFT` down, pointer does not move |
| wheel / pan | `REL_WHEEL` +1 / −1, `REL_HWHEEL` +1, pointer does not move (scroll-only) |
| wheel + position in one frame | `REL_WHEEL` 2 **and** `ABS_X/Y` updated |
| relative pointer (`0x8557`) | `REL_X`/`REL_Y` deltas and `REL_WHEEL` on the relative input device |
| system key (`0x8657`, power down) | `KEY_POWER` press **and** automatic release on the System Control device |
| cross-machine run | HID port on a second PC, CDC on the first: one `0x8657 code=1` frame put the target to sleep/shut down |
| frame split across writes | reassembled correctly |
| 3 frames in one write | all applied, last one wins |
| 500-frame burst (6012 B, blocking write) | no loss, final position exact |
| garbage before a valid frame | re-synchronised within one frame |
