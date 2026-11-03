#!/usr/bin/env python3
"""
Host-side smoke test for examples/dual/hid_cdc_bridge.

Sends 12-byte frames to the bridge's CDC port and checks that the HID side
(the same PC) reports the expected keyboard/mouse/system events.

Usage:
  ./bridge_check.py /dev/ttyACM2            # auto-picks /dev/ttyACM* if omitted
  ./bridge_check.py --no-evdev /dev/ttyACM2 # only send frames, skip checking

Requires read access to /dev/input/eventX (member of the "input" group).
"""

import argparse
import fcntl
import glob
import os
import select
import struct
import sys
import termios
import threading
import time
import tty

FRAME = struct.Struct("<H8sH")
TAIL = 0x0900
HDR_KEY = 0x8257
HDR_MOUSE_MOVE = 0x8357
HDR_MOUSE_BUTTON = 0x8457
HDR_MOUSE_REL = 0x8557
HDR_SYSTEM_KEY = 0x8657
HDR_STATUS = 0x8757  # device -> host: data[0] = LED bitmap
LED_NUM = 0x01
LED_CAPS = 0x02
LED_OFFSET = 7  # frame[2 + 5]: bridge_status_t.leds
KEY_NUM_LOCK = 0x53  # HID usage, toggles the host's num lock

SYS_POWER_DOWN = 1

EV_SYN, EV_KEY, EV_REL, EV_ABS = 0x00, 0x01, 0x02, 0x03
ABS_X, ABS_Y = 0x00, 0x01
REL_HWHEEL, REL_WHEEL = 0x06, 0x08
KEY_A = 30     # HID keyboard usage 0x04
KEY_POWER = 116
BTN_LEFT = 0x110

EVIOCGRAB = 0x40044590
EVIOCGABS = 0x80184540  # EVIOCGABS(0), axis argument is added per axis

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


def system_key_frame(code: int) -> bytes:
    return frame(HDR_SYSTEM_KEY, bytes([code]) + bytes(7))


def find_cdc() -> str:
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        sys.exit("no /dev/ttyACM* found; is the CDC port connected?")
    return candidates[-1]


class Evdev:
    """Event device lookup by name, optionally filtered on absolute axes.

    The absolute and relative pointer are both called "<product> Mouse"; only
    their capabilities tell them apart (the relative one has no ABS axes).
    """

    def __init__(self, match: str, need_abs=None):
        self.path = None
        for name_path in glob.glob("/sys/class/input/event*/device/name"):
            try:
                with open(name_path) as f:
                    name = f.read().strip()
            except OSError:
                continue
            if match not in name:
                continue
            sysdir = os.path.dirname(name_path)
            try:
                with open(os.path.join(sysdir, "capabilities/abs")) as f:
                    has_abs = f.read().strip() != "0"
            except OSError:
                has_abs = False
            if need_abs is True and not has_abs:
                continue
            if need_abs is False and has_abs:
                continue
            self.path = "/dev/input/" + name_path.split("/")[4]
            break
        if self.path is None:
            raise RuntimeError(f"no event device matching {match!r} (need_abs={need_abs})")
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)

    def abs_value(self, axis: int):
        """Current value of an absolute axis (kernel emits nothing when it does
        not change, so tests must pick a target different from the current one)."""
        import array
        buf = array.array("i", [0] * 5)
        fcntl.ioctl(self.fd, EVIOCGABS + axis, buf, True)
        return buf[0]

    def grab(self, on: bool):
        fcntl.ioctl(self.fd, EVIOCGRAB, 1 if on else 0)

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


class StatusReader(threading.Thread):
    """Collects device -> host status frames (header 0x8757) from the CDC port."""

    def __init__(self, fd):
        super().__init__(daemon=True)
        self.fd = fd
        self.buf = bytearray()
        self.frames = []
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if not r:
                continue
            try:
                data = os.read(self.fd, 256)
            except OSError:
                continue
            self.buf += data
            while len(self.buf) >= 12:
                header = self.buf[0] | (self.buf[1] << 8)
                tail = self.buf[10] | (self.buf[11] << 8)
                if header == HDR_STATUS and tail == TAIL:
                    self.frames.append(self.buf[LED_OFFSET])
                    del self.buf[:12]
                else:
                    del self.buf[:1]

    def wait_leds(self, mask, value, timeout=3.0):
        """Wait for a status frame whose LED bits match mask==value."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            while self.frames:
                leds = self.frames.pop(0)
                if (leds & mask) == value:
                    return leds
            time.sleep(0.02)
        return None

    def wait_frame(self, timeout=3.0):
        """LED bitmap of the next status frame, whatever it says."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.frames:
                return self.frames.pop(0)
            time.sleep(0.02)
        return None


def configure_port(fd):
    """Put the tty in raw mode and turn off flow control.

    Without this the tty keeps its default termios: IXON interprets 0x11/0x13
    bytes inside the binary frames as XON/XOFF and stops delivering data, and
    CRTSCTS can stall reads too. minicom does this for you; cat does not, which
    is why `cat /dev/ttyACM1` appears to work and then goes silent.
    """
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~termios.CRTSCTS
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


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

    kbd = abs_mouse = rel_mouse = system = None
    if not args.no_evdev:
        kbd = Evdev("HID Bridge Keyboard")
        abs_mouse = Evdev("HID Bridge Mouse", need_abs=True)
        rel_mouse = Evdev("HID Bridge Mouse", need_abs=False)
        system = Evdev("System Control")
        print(f"keyboard : {kbd.path}")
        print(f"mouse abs: {abs_mouse.path}")
        print(f"mouse rel: {rel_mouse.path}")
        print(f"system   : {system.path}")

    fd = os.open(cdc, os.O_RDWR | os.O_NOCTTY)
    configure_port(fd)
    os.set_blocking(fd, False)
    status = StatusReader(fd)
    status.start()
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

        # 2. absolute mouse move: 2048/4096 -> ~16384/32767 (pick a target that
        #    differs from the current position, or the kernel emits no event)
        target = 1024 if (abs_mouse and abs_mouse.abs_value(ABS_X) == 16384) else 2048
        print(f"mouse move to {target}/4096")
        if abs_mouse:
            abs_mouse.read(0.1)
        os.write(fd, mouse_frame(HDR_MOUSE_MOVE, 0, target, target))
        if abs_mouse:
            ev = abs_mouse.read(1.0)
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            expecting = target * 8
            failures += expect(abs(pos.get(ABS_X, -1) - expecting) < 200,
                               f"ABS_X ~{expecting} (got {pos.get(ABS_X)})")
            failures += expect(abs(pos.get(ABS_Y, -1) - expecting) < 200,
                               f"ABS_Y ~{expecting} (got {pos.get(ABS_Y)})")

        # 3. absolute button-only frame must not move the pointer
        print("mouse button press (position must not jump)")
        if abs_mouse:
            abs_mouse.read(0.1)
        os.write(fd, mouse_frame(HDR_MOUSE_BUTTON, 0x01, 0, 0))
        if abs_mouse:
            ev = abs_mouse.read(1.0)
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            keys = [(c, v) for (t, c, v) in ev if t == EV_KEY]
            failures += expect((BTN_LEFT, 1) in keys, f"BTN_LEFT down seen (got {keys})")
            failures += expect(ABS_X not in pos or abs(pos[ABS_X] - expecting) < 200,
                               f"ABS_X unchanged (got {pos.get(ABS_X)})")
        os.write(fd, mouse_frame(HDR_MOUSE_BUTTON, 0x00, 0, 0))
        if abs_mouse:
            abs_mouse.read(0.5)

        # 4. absolute wheel / pan (relative, carried in the mouse-move frame)
        print("absolute wheel / pan")
        for wheel, pan, expect_wheel, expect_pan in ((1, 0, 1, 0), (-1, 0, -1, 0), (0, 1, 0, 1)):
            if abs_mouse:
                abs_mouse.read(0.1)
            os.write(fd, mouse_frame(HDR_MOUSE_MOVE, 0, 0, 0, wheel, pan))
            if abs_mouse:
                ev = abs_mouse.read(1.0)
                rel = {c: v for (t, c, v) in ev if t == EV_REL}
                pos = {c: v for (t, c, v) in ev if t == EV_ABS}
                failures += expect(rel.get(REL_WHEEL, 0) == expect_wheel,
                                   f"REL_WHEEL {expect_wheel} (got {rel.get(REL_WHEEL)})")
                failures += expect(rel.get(REL_HWHEEL, 0) == expect_pan,
                                   f"REL_HWHEEL {expect_pan} (got {rel.get(REL_HWHEEL)})")
                failures += expect(ABS_X not in pos, "pointer must not move on scroll-only frame")

        # 5. relative pointer: deltas, wheel and buttons
        print("relative pointer")
        for dx, dy, wheel, expect_dx, expect_dy in ((30, -20, 0, 30, -20), (-5, 7, 0, -5, 7)):
            if rel_mouse:
                rel_mouse.read(0.1)
            os.write(fd, mouse_frame(HDR_MOUSE_REL, 0, dx, dy, wheel))
            if rel_mouse:
                ev = rel_mouse.read(1.0)
                rel = {c: v for (t, c, v) in ev if t == EV_REL}
                failures += expect(rel.get(0x00) == expect_dx, f"REL_X {expect_dx} (got {rel.get(0x00)})")
                failures += expect(rel.get(0x01) == expect_dy, f"REL_Y {expect_dy} (got {rel.get(0x01)})")

        print("relative wheel")
        if rel_mouse:
            rel_mouse.read(0.1)
        os.write(fd, mouse_frame(HDR_MOUSE_REL, 0, 0, 0, 2))
        if rel_mouse:
            ev = rel_mouse.read(1.0)
            rel = {c: v for (t, c, v) in ev if t == EV_REL}
            failures += expect(rel.get(REL_WHEEL, 0) == 2, f"REL_WHEEL 2 (got {rel.get(REL_WHEEL)})")

        # 6. system control: power down is a momentary key
        print("system control key (grabbed, cannot reach the desktop)")
        if system:
            system.read(0.1)
            system.grab(True)
        try:
            os.write(fd, system_key_frame(SYS_POWER_DOWN))
            if system:
                ev = system.read(1.0)
                keys = [(c, v) for (t, c, v) in ev if t == EV_KEY]
                failures += expect((KEY_POWER, 1) in keys, f"KEY_POWER down seen (got {keys})")
                failures += expect((KEY_POWER, 0) in keys, f"KEY_POWER auto-release seen (got {keys})")
        finally:
            if system:
                system.grab(False)

        # 7. Lock-key state is reported back over CDC (status frame 0x8757),
        #    using Num Lock: the host always has it on, so both the "on" and the
        #    "off" state are exercised and each toggle must produce a frame.
        print("num lock state reported back")
        status.frames.clear()
        os.write(fd, key_frame(0, [KEY_NUM_LOCK]))
        time.sleep(0.2)
        os.write(fd, key_frame(0, []))
        first = status.wait_frame()
        failures += expect(first is not None, f"status frame after num lock toggle (got {first})")

        os.write(fd, key_frame(0, [KEY_NUM_LOCK]))
        time.sleep(0.2)
        os.write(fd, key_frame(0, []))
        second = status.wait_frame()
        failures += expect(second is not None, f"status frame after second toggle (got {second})")
        if first is not None and second is not None:
            failures += expect((first ^ second) & LED_NUM,
                               f"num lock bit toggled (first=0x{first:02x} second=0x{second:02x})")

        # 8. resynchronisation: garbage in front of a valid frame
        print("resync after garbage")
        if abs_mouse:
            abs_mouse.read(0.1)
        os.write(fd, b"\xff\x00\x57\x82\xaa" + mouse_frame(HDR_MOUSE_MOVE, 0, 1024, 3072))
        if abs_mouse:
            ev = abs_mouse.read(1.0)
            pos = {c: v for (t, c, v) in ev if t == EV_ABS}
            failures += expect(abs(pos.get(ABS_X, -1) - 8192) < 200, f"ABS_X ~8192 (got {pos.get(ABS_X)})")
    finally:
        status.stop.set()
        status.join(timeout=1.0)
        os.close(fd)
        for dev in (kbd, abs_mouse, rel_mouse, system):
            if dev:
                dev.close()

    print("RESULT:", "PASS" if failures == 0 else f"{failures} FAILURE(S)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
