#!/usr/bin/env python3
"""
Interactive (curses) terminal for manually exercising the HID CDC bridge.

Type on the keyboard shown in the TUI and the machine attached to the bridge's
HID port receives the keystrokes; mouse/system commands are single keys. Every
status frame the bridge sends back is decoded in the log pane.

Usage:
  ./cdc_tui.py                 # auto-detect the CDC port (cafe:4001)
  ./cdc_tui.py /dev/ttyACM2    # or name it
  ./cdc_tui.py --selftest      # no UI: send a few frames, print what came back

Frame format (see ../README.md): 12 bytes, little endian
  header | payload[8] | tail 0x0900
  host -> device: 0x8257 keyboard, 0x8357 abs mouse, 0x8457 mouse buttons,
                  0x8557 relative mouse, 0x8657 system key
  device -> host: 0x8757 status (state, hid_itf_count, uptime, flags, leds)
"""

import argparse
import curses
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
HDR_MOUSE_ABS = 0x8357
HDR_MOUSE_BTN = 0x8457
HDR_MOUSE_REL = 0x8557
HDR_SYSTEM = 0x8657
HDR_STATUS = 0x8757
HDR_STATUS_REQ = 0x8758

SYS_POWER, SYS_SLEEP, SYS_WAKE = 1, 2, 3

STATE_NAMES = {0: "unmounted", 1: "active", 2: "suspended"}
LED_NUM, LED_CAPS, LED_SCROLL = 0x01, 0x02, 0x04

# ASCCI -> (modifier, HID usage)
KEYMAP = {}
for _c in range(ord("a"), ord("z") + 1):
    KEYMAP[chr(_c)] = (0, 0x04 + (_c - ord("a")))
for _c in range(ord("1"), ord("9") + 1):
    KEYMAP[chr(_c)] = (0, 0x1E + (_c - ord("1")))
KEYMAP["0"] = (0, 0x27)
KEYMAP.update({
    " ": (0, 0x2C), "\n": (0, 0x28), "\t": (0, 0x2B), "\b": (0, 0x2A),
    "-": (0, 0x2D), "=": (0, 0x2E), "[": (0, 0x2F), "]": (0, 0x30),
    "\\": (0, 0x31), ";": (0, 0x33), "'": (0, 0x34), "`": (0, 0x35),
    ",": (0, 0x36), ".": (0, 0x37), "/": (0, 0x38),
    "!": (0x02, 0x1E), "@": (0x02, 0x1F), "#": (0x02, 0x20), "$": (0x02, 0x21),
    "%": (0x02, 0x22), "^": (0x02, 0x23), "&": (0x02, 0x24), "*": (0x02, 0x25),
    "(": (0x02, 0x26), ")": (0x02, 0x27), "_": (0x02, 0x2D), "+": (0x02, 0x2E),
    "{": (0x02, 0x2F), "}": (0x02, 0x30), "|": (0x02, 0x31), ":": (0x02, 0x33),
    '"': (0x02, 0x34), "~": (0x02, 0x35), "<": (0x02, 0x36), ">": (0x02, 0x37),
    "?": (0x02, 0x38),
})


def frame(header, payload):
    return FRAME.pack(header, payload, TAIL)


def key_frame(modifier, usage):
    return frame(HDR_KEY, bytes([modifier, 0, usage]) + bytes(5))


def key_release():
    return frame(HDR_KEY, bytes(8))


def abs_frame(x, y, buttons=0, wheel=0):
    return frame(HDR_MOUSE_ABS, struct.pack("<Bbhhbb", buttons, 0, x, y, wheel, 0))


def rel_frame(dx, dy, wheel=0):
    return frame(HDR_MOUSE_REL, struct.pack("<Bbhhbb", 0, 0, dx, dy, wheel, 0))


def system_frame(code):
    return frame(HDR_SYSTEM, bytes([code]) + bytes(7))


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


def find_port():
    for tty in sorted(glob.glob("/dev/ttyACM*")):
        for prop in glob.glob("/sys/class/tty/%s/device/../idVendor" % os.path.basename(tty)):
            try:
                if open(prop).read().strip() == "cafe":
                    return tty
            except OSError:
                pass
    return "/dev/ttyACM1"


def decode_status(f):
    if len(f) < 12 or f[0] | (f[1] << 8) != HDR_STATUS:
        return None
    state, itfs, uptime, flags, leds = struct.unpack_from("<BBHBB", f, 2)
    return {
        "state": STATE_NAMES.get(state, f"state{state}"),
        "itfs": itfs,
        "uptime": uptime,
        "dtr": bool(flags & 1),
        "leds": leds,
    }


class Link:
    """Owns the CDC port: writes frames, parses the incoming byte stream."""

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        configure_port(self.fd)
        os.set_blocking(self.fd, False)
        self.lock = threading.Lock()
        self.buf = bytearray()
        self.log = []
        self.last_status = None
        self.last_raw = None
        self.pending_query = None   # (token, time) of the last query we sent
        self.rx_frames = 0
        self.tx_frames = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while not self.stop.is_set():
            try:
                r, _, _ = select.select([self.fd], [], [], 0.05)
            except (OSError, ValueError):
                return
            if not r:
                continue
            try:
                data = os.read(self.fd, 256)
            except OSError:
                continue
            self.buf += data
            while len(self.buf) >= 12:
                if (self.buf[0] | (self.buf[1] << 8)) == HDR_STATUS and \
                        (self.buf[10] | (self.buf[11] << 8)) == TAIL:
                    frm = bytes(self.buf[:12])
                    del self.buf[:12]
                    self.rx_frames += 1
                    st = decode_status(frm)
                    if st:
                        self.last_status = st
                        self.last_raw = frm
                        if self.pending_query and self.pending_query[0] == frm[8]:
                            dt = (time.time() - self.pending_query[1]) * 1000.0
                            self.pending_query = None
                            self.add_log(f"query token {frm[8]} answered in {dt:.1f} ms")
                        self.add_log(self._describe_status(st))
                else:
                    del self.buf[:1]

    @staticmethod
    def _describe_status(st):
        leds = st["leds"]
        return ("status: %-9s itf=%d up=%us dtr=%s leds[num=%d caps=%d scr=%d]"
                % (st["state"], st["itfs"], st["uptime"], "y" if st["dtr"] else "n",
                   leds & LED_NUM and 1 or 0, leds & LED_CAPS and 1 or 0, leds & LED_SCROLL and 1 or 0))

    def add_log(self, text):
        self.log.append((time.strftime("%H:%M:%S"), text))
        del self.log[:-500]

    def query(self):
        token = (self.tx_frames + 1) & 0xFF
        self.pending_query = (token, time.time())
        return self.send(frame(HDR_STATUS_REQ, bytes([token]) + bytes(7)),
                         f"status query token {token}")

    def show_last_raw(self):
        if self.last_raw is None:
            self.add_log("no status frame received yet")
        else:
            self.add_log("raw: " + " ".join(f"{b:02x}" for b in self.last_raw))

    def send(self, raw, note):
        with self.lock:
            try:
                n = os.write(self.fd, raw)
            except OSError as exc:
                self.add_log(f"write failed: {exc}")
                return False
        if n == len(raw):
            self.tx_frames += 1
            self.add_log(f"tx {note}")
            return True
        self.add_log(f"short write {n}/{len(raw)}: {note}")
        return False

    def close(self):
        self.stop.set()
        self.thread.join(timeout=0.5)
        try:
            os.close(self.fd)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# curses UI
# ---------------------------------------------------------------------------

HELP = [
    ("a-z 0-9 / punctuation", "type on the target machine"),
    ("Enter  Tab  Space  Backspace", "as labelled"),
    ("Arrow keys", "relative mouse move (4 units)"),
    ("m", "absolute mouse to screen centre"),
    ("w / s", "wheel up / wheel down"),
    ("c", "toggle Caps Lock"),
    ("p / l / k", "system: power down / sleep / wake"),
    ("r", "reset: relative pointer to the top-left corner"),
    (":hex ...", "send raw bytes, e.g. :hex 57 82 04 00 00 00 00 00 00 00 00 09"),
    (":abs X Y", "absolute move, X/Y in 0..4096"),
    (":rel DX DY [W]", "relative move / scroll"),
    ("t", "ask the bridge for its state now (query + token)"),
    ("i", "show the raw hex of the last status frame"),
    ("q", "quit"),
]


def draw(stdscr, link, prompt):
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    st = link.last_status
    title = f"HID CDC bridge TUI  {link.path}  tx={link.tx_frames} rx={link.rx_frames}"
    stdscr.attron(curses.A_REVERSE)
    stdscr.addnstr(0, 0, title.ljust(w - 1), w - 1)
    stdscr.attroff(curses.A_REVERSE)

    if st:
        line = ("state=%s  HID itfs=%d  uptime=%us  host DTR=%s  LEDs: num=%d caps=%d scroll=%d"
                % (st["state"], st["itfs"], st["uptime"], "yes" if st["dtr"] else "no",
                   st["leds"] & LED_NUM and 1 or 0, st["leds"] & LED_CAPS and 1 or 0,
                   st["leds"] & LED_SCROLL and 1 or 0))
    else:
        line = "no status frame received yet (is the HID side enumerated?)"
    stdscr.addnstr(1, 0, line.ljust(w - 1), w - 1)

    split = max(20, w // 2)
    stdscr.addnstr(2, 0, " log ".center(split - 1, "-"), split - 1)
    stdscr.addnstr(2, split, " keys ".center(w - split - 1, "-"), w - split - 2)

    rows = h - 5
    entries = link.log[-rows:]
    for i, (ts, text) in enumerate(entries):
        stdscr.addnstr(3 + i, 0, f"{ts} {text}", split - 1)

    for i, (keys, desc) in enumerate(HELP[:rows]):
        stdscr.addnstr(3 + i, split, f"{keys:<28} {desc}", w - split - 2)

    stdscr.addnstr(h - 2, 0, ":" + prompt if prompt else "press a key, or ':' for a raw command", w - 1)
    stdscr.refresh()


def run_ui(stdscr, link):
    curses.curs_set(0)
    stdscr.timeout(200)  # getch wakes every 200ms (returns -1) so incoming status frames redraw continuously
    prompt = ""
    draw(stdscr, link, prompt)
    while True:
        ch = stdscr.getch()

        if prompt:
            if ch in (curses.KEY_ENTER, 10, 13):
                cmd, prompt = prompt.strip(), ""
                handle_command(link, cmd)
            elif ch in (27,):
                prompt = ""
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                prompt = prompt[:-1]
            elif 32 <= ch < 127:
                prompt += chr(ch)
            draw(stdscr, link, prompt)
            continue

        if ch in (ord("q"), ord("Q"), 27):
            return
        if ch == ord(":"):
            prompt = ":"
            draw(stdscr, link, prompt)
            continue

        if ch == curses.KEY_UP:
            link.send(rel_frame(0, -4), "rel 0,-4")
        elif ch == curses.KEY_DOWN:
            link.send(rel_frame(0, 4), "rel 0,+4")
        elif ch == curses.KEY_LEFT:
            link.send(rel_frame(-4, 0), "rel -4,0")
        elif ch == curses.KEY_RIGHT:
            link.send(rel_frame(4, 0), "rel +4,0")
        elif ch in (ord("m"),):
            link.send(abs_frame(2048, 2048), "abs 2048,2048")
        elif ch == ord("w"):
            link.send(rel_frame(0, 0, wheel=1), "wheel +1")
        elif ch == ord("s"):
            link.send(rel_frame(0, 0, wheel=-1), "wheel -1")
        elif ch == ord("c"):
            link.send(key_frame(0, 0x39), "Caps Lock"); link.send(key_release(), "release")
        elif ch == ord("p"):
            link.send(system_frame(SYS_POWER), "system power down")
        elif ch == ord("l"):
            link.send(system_frame(SYS_SLEEP), "system sleep")
        elif ch == ord("k"):
            link.send(system_frame(SYS_WAKE), "system wake up")
        elif ch == ord("t"):
            link.query()
        elif ch == ord("i"):
            link.show_last_raw()
        elif ch == ord("r"):
            for _ in range(8):
                link.send(rel_frame(-127, -127), "rel -127,-127 (corner)")
        else:
            mapped = KEYMAP.get(chr(ch)) if 32 <= ch < 127 else None
            if mapped:
                mod, usage = mapped
                link.send(key_frame(mod, usage), f"key usage 0x{usage:02x}")
                link.send(key_release(), "release")
        draw(stdscr, link, prompt)


def handle_command(link, cmd):
    if not cmd:
        return
    if cmd.startswith(":"):
        cmd = cmd[1:]
    parts = cmd.split()
    if not parts:
        return
    try:
        if parts[0] == "hex":
            raw = bytes(int(b, 16) for b in parts[1:])
            link.send(raw, f"raw {len(raw)} bytes")
        elif parts[0] == "abs" and len(parts) >= 3:
            link.send(abs_frame(int(parts[1]), int(parts[2])), f"abs {parts[1]},{parts[2]}")
        elif parts[0] == "rel":
            wheel = int(parts[3]) if len(parts) > 3 else 0
            link.send(rel_frame(int(parts[1]), int(parts[2]), wheel), f"rel {parts[1]},{parts[2]} w{wheel}")
        else:
            link.add_log(f"unknown command: {cmd}")
    except (ValueError, IndexError):
        link.add_log(f"bad command: {cmd}")


# ---------------------------------------------------------------------------
# headless self test
# ---------------------------------------------------------------------------


def selftest(link):
    print(f"port: {link.path}")
    time.sleep(1.0)  # let the reader (and the bridge's heartbeat) start

    steps = [
        ("keyboard 'a'", [key_frame(0, 0x04), key_release()]),
        ("absolute centre", [abs_frame(2048, 2048)]),
        ("relative +8,-8", [rel_frame(8, -8)]),
        ("wheel +1", [rel_frame(0, 0, wheel=1)]),
    ]
    failures = 0
    for name, frames in steps:
        before = link.rx_frames
        for f in frames:
            link.send(f, name)
        deadline = time.time() + 5.0
        while time.time() < deadline and link.rx_frames == before:
            time.sleep(0.05)
        got = link.rx_frames > before
        print(f"  {'PASS' if got else 'FAIL'}  {name}: status frame {'received' if got else 'missing'}")
        failures += 0 if got else 1

    print(f"  tx={link.tx_frames} rx={link.rx_frames}")
    if link.last_status:
        print("  last:", link._describe_status(link.last_status))
    return 0 if failures == 0 else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    port = args.port or find_port()
    try:
        link = Link(port)
    except OSError as exc:
        sys.exit(f"cannot open {port}: {exc}")

    try:
        if args.selftest:
            return selftest(link)
        curses.wrapper(run_ui, link)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
