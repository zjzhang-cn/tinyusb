#!/usr/bin/env python3
"""
Host-side smoke test for examples/dual/hid_cdc_bridge.

Sends 12-byte frames to the bridge's CDC port and checks that the HID side
(the same PC) reports the expected keyboard/mouse events.

Usage:
  ./bridge_check.py /dev/ttyACM2            # auto-picks /dev/ttyACM* if omitted
  ./bridge_check.py --no-evdev /dev/ttyACM2 # only send frames, skip checking

Requires read access to /dev/input/eventX (member of the "input" group).
"""

import argparse
import glob
import os
import select
import struct
import sys
import time

FRAME = struct.Struct("<H8sH")
TAIL = 0x0900
HDR_KEY = 0x8257
HDR_MOUSE_MOVE = 0x8357
HDR_MOUSE_BUTTON = 0x8457

EV_SYN, EV_KEY, EV_REL, EV_ABS = 0x00, 0x01, 0x02, 0x03
ABS_X, ABS_Y = 0x00, 0x01
REL_HWHEEL, REL_WHEEL = 0x06, 0x08
KEY_A = 30  # HID keyboard usage 0x04
SYN_REPORT = 0

EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)


def frame(header, payload: bytes) -> bytes:
    assert len(payload) == 8
    return FRAME.pack(header, payload, TAIL)


def key_frame(modifier: int, keycodes) -> bytes:
    keys = bytes(keycodes) + bytes(6 - len(keycodes))
    return frame(HDR_KEY, bytes([modifier, 0]) + keys)


def mouse_frame(header, buttons, x, y, wheel=0, pan=0) -> bytes:
    return frame(header, struct.pack("<Bbhhbb", buttons, 0, x, y, wheel, pan))


def find_cdc() -> str:
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        sys.exit("no /dev/ttyACM* found; is the CDC port connected?")
    return candidates[-1]


class Evdev:
    def __init__(self, match: str):
        self.path = None
        for name_path in glob.glob("/sys/class/input/event*/device/name"):
            try:
                with open(name_path) as f:
                    name = f.read().strip()
            except OSError:
                continue
            if match in name:
                self.path = "/dev/input/" + name_path.split("/")[4]
                break
        if self.path is None:
            raise RuntimeError(f"no event device matching {match!r}")
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)

    def read(self, timeout=1.0):
        events = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], max(0.0, deadline - time.time()))
            if not r:
                break
            data = os.read(self.fd, EVENT_SIZE * 64)
            for i in range(0, len(data), EVENT_SIZE):
                sec, usec, etype, code, value = struct.unpack(EVENT_FMT, data[i:i + EVENT_SIZE])
                events.append((etype, code, value))
        return events

    def close(self):
        os.close(self.fd)


def expect(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    return 0 if cond else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cdc", nargs="?")
    ap.add_argument("--no-evdev", action="store_true")
    args = ap.parse_args()

    cdc = args.cdc or find_cdc()
    print(f"CDC port : {cdc}")

    kbd = mouse = None
    if not args.no_evdev:
        kbd = Evdev("HID CDC Bridge Keyboard")
        mouse = Evdev("HID CDC Bridge Mouse")
        print(f"keyboard : {kbd.path}")
        print(f"mouse    : {mouse.path}")

    fd = os.open(cdc, os.O_RDWR | os.O_NOCTTY)
    os.set_blocking(fd, False)
    failures = 0
    try:
        # 1. keyboard: press 'a' (HID usage 0x04 -> Linux KEY_A == 30)
        print("keyboard 'a'")
        if kbd:
            kbd.read(0.1)  # drain
        os.write(fd, key_frame(0, [0x04]))
        if kbd:
            ev = kbd.read(1.0)
            keys = [(c, v) for (t, c, v) in ev if t == EV_KEY]
            failures += expect((KEY_A, 1) in keys, f"KEY_A down seen (got {keys})")
        os.write(fd, key_frame(0, []))
        if kbd:
            ev = kbd.read(1.0)
            keys = [(c, v) for (t, c, v) in ev if t == EV_KEY]
            failures += expect((KEY_A, 0) in keys, f"KEY_A up seen (got {keys})")

        # 2. absolute mouse move to the centre: 2048/4096 -> ~16383/32767
        print("mouse move to centre")
        if mouse:
            mouse.read(0.1)
        os.write(fd, mouse_frame(HDR_MOUSE_MOVE, 0, 2048, 2048))
        if mouse:
            ev = mouse.read(1.0)
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            failures += expect(abs(pos.get(ABS_X, -1) - 16384) < 200, f"ABS_X ~16384 (got {pos.get(ABS_X)})")
            failures += expect(abs(pos.get(ABS_Y, -1) - 16384) < 200, f"ABS_Y ~16384 (got {pos.get(ABS_Y)})")

        # 3. button-only frame must not move the pointer
        print("mouse button press (position must not jump)")
        if mouse:
            mouse.read(0.1)
        os.write(fd, mouse_frame(HDR_MOUSE_BUTTON, 0x01, 0, 0))
        if mouse:
            ev = mouse.read(1.0)
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            keys = [(c, v) for (t, c, v) in ev if t == EV_KEY]
            failures += expect((0x110, 1) in keys, f"BTN_LEFT down seen (got {keys})")
            failures += expect(ABS_X not in pos or abs(pos[ABS_X] - 16384) < 200,
                               f"ABS_X unchanged (got {pos.get(ABS_X)})")
        os.write(fd, mouse_frame(HDR_MOUSE_BUTTON, 0x00, 0, 0))
        if mouse:
            mouse.read(0.5)

        # 4. wheel / pan (relative, carried in the mouse-move frame)
        print("wheel up / down, horizontal pan")
        for wheel, pan, expect_wheel, expect_pan in ((1, 0, 1, 0), (-1, 0, -1, 0), (0, 1, 0, 1)):
            if mouse:
                mouse.read(0.1)
            os.write(fd, mouse_frame(HDR_MOUSE_MOVE, 0, 0, 0, wheel, pan))
            if mouse:
                ev = mouse.read(1.0)
                rel = {c: v for (t, c, v) in ev if t == EV_REL}
                pos = {c: v for (t, c, v) in ev if t == EV_ABS}
                failures += expect(rel.get(REL_WHEEL, 0) == expect_wheel,
                                   f"REL_WHEEL {expect_wheel} (got {rel.get(REL_WHEEL)})")
                failures += expect(rel.get(REL_HWHEEL, 0) == expect_pan,
                                   f"REL_HWHEEL {expect_pan} (got {rel.get(REL_HWHEEL)})")
                failures += expect(ABS_X not in pos, "pointer must not move on scroll-only frame")

        # 5. scroll together with a position update
        print("scroll + move in one frame")
        if mouse:
            mouse.read(0.1)
        os.write(fd, mouse_frame(HDR_MOUSE_MOVE, 0, 3072, 1024, 2, 0))
        if mouse:
            ev = mouse.read(1.0)
            rel = {c: v for (t, c, v) in ev if t == EV_REL}
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            failures += expect(rel.get(REL_WHEEL, 0) == 2, f"REL_WHEEL 2 (got {rel.get(REL_WHEEL)})")
            failures += expect(abs(pos.get(ABS_X, -1) - 24576) < 200, f"ABS_X ~24576 (got {pos.get(ABS_X)})")
            failures += expect(abs(pos.get(ABS_Y, -1) - 8192) < 200, f"ABS_Y ~8192 (got {pos.get(ABS_Y)})")

        # 6. resynchronisation: garbage in front of a valid frame
        print("resync after garbage")
        if mouse:
            mouse.read(0.1)
        os.write(fd, b"\xff\x00\x57\x82\xaa" + mouse_frame(HDR_MOUSE_MOVE, 0, 1024, 3072))
        if mouse:
            ev = mouse.read(1.0)
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            failures += expect(abs(pos.get(ABS_X, -1) - 8192) < 200, f"ABS_X ~8192 (got {pos.get(ABS_X)})")
    finally:
        os.close(fd)
        if kbd:
            kbd.close()
        if mouse:
            mouse.close()

    print("RESULT:", "PASS" if failures == 0 else f"{failures} FAILURE(S)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
