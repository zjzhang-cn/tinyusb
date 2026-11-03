# CDC device → host: the "unreliable IN channel" was a host termios trap

Status: **resolved**. The bridge's device → host path works. The failures that
looked like a firmware problem came from the host side: a tty opened with its
default termios enables software flow control (`IXON`), and a `0x13` byte inside
a binary frame is then treated as XOFF, which stops delivery until a `0x11`
(XON) happens to arrive. `CRTSCTS` and `ICRNL` make it worse.

`minicom` sets raw mode on open, so it always worked; `cat` does not, so it
"works for a while and then goes silent" — exactly the symptom that sent this
investigation down a firmware rabbit hole.

## Evidence

Same board, same firmware, only the host's serial settings changed:

| Host termios | Bytes received in 10 s |
|---|---|
| `raw -ixon -ixoff -crtscts` | **144** (12 status frames) |
| default (`stty sane`) | **0** |

With raw mode, the host-initiated query answers every time:

```
queries answered: 10/10   latency p50 0.1 ms
selftest 6 consecutive runs: 4/4 each
```

The device-side counters agree (`cdc-tx: arms≈done`, `stuck=0`): the controller
was always arming and completing the transfers.

## Why the earlier measurements looked like a firmware bug

`uptime_s` in the status frame increments once per second, so its low byte
walks through every value, including `0x13` (XOFF) and `0x11` (XON). A `cat`
that starts reading right after a reset therefore shows data until the first
`0x13` lands in the stream, then nothing — which reads exactly like "the
channel dies after a few seconds". Repeated open/close cycles only changed
where in that cycle the observation started.

Every "0/N" result in this log was taken through a reader without raw mode.

## The one real device-side constraint (still valid)

A 12-byte frame is a short packet. The controller reports its completion only
when the host issues one more IN token, so if the host stops reading while a
frame is armed, that frame is never retired and the endpoint stays pending.

The pump handles this by staying serialized on the completion (never re-arming
over a live transfer) with a 1 s safety valve that resets the endpoint when the
host has gone away:

```c
if (dev.tx_busy) {
  if ((int32_t)(now - tx_next_ms) < TX_PENDING_TIMEOUT_MS) return;
  usbfs_dcd_edpt_reset(RHPORT, EP_DATA_IN);   // 1 s, host stopped reading
  dev.tx_busy = false;
}
```

A healthy host completes a frame in milliseconds, so the valve never fires in
normal use.

## Rules for host-side tools

Any program that opens the CDC port for binary data must:

```python
tty.setraw(fd)                              # no ICANON/ICRNL/OPOST
attrs = termios.tcgetattr(fd)
attrs[2] &= ~termios.CRTSCTS                # no hardware flow control
attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)   # no XON/XOFF
termios.tcsetattr(fd, termios.TCSANOW, attrs)
```

`tools/cdc_tui.py` and `tools/bridge_check.py` do this in `configure_port()`.
`minicom` does it internally. `cat`, `screen` with default settings, and most
naive serial snippets do not — use `stty -F /dev/ttyACM1 raw -crtscts -ixon`
first, or expect the "works then dies" behaviour.

## What the journey was worth keeping

| Attempt | What it actually measured |
|---|---|
| WCH `SimulateCDC` driver | genuinely broken device → host (EP3 IN never transmitted); replaced |
| TinyUSB DCD copy + mini stack | correct replacement; enumeration and both directions work |
| Wait-for-completion pump | right model; the "silence" was the host's XOFF |
| fire-and-forget pump | wrong: re-armed over a live transfer; replaced by the serialized pump |
| endpoint reset on DTR edges / idle | kept as the safety valve, now scoped to a 1 s timeout |
| query `0x8758` + token echo | works; p50 0.1 ms with a raw-mode reader |
| pad frames to 64 bytes | not needed once the host reads in raw mode; reverted |
| 250 ms heartbeat | removed: report on state change or on query instead |

## Verified by bisecting the branch

With the host tool fixed (raw mode), **every commit from `ec07e2172` onwards
passes the same 4-check selftest, twice each**:

| commit | content | selftest |
|---|---|---|
| `ec07e2172` | TinyUSB DCD + mini stack (wait-for-completion pump) | 4 4 |
| `be2ad61da` | host-initiated query `0x8758` | 4 4 |
| `9cc3abeda` | enumerated gate + 250 ms heartbeat | 4 4 |
| `9e4ec0c4d` | serialized pump + 1 s safety valve | 4 4 |
| `faa40c908` | raw-mode tools, no heartbeat | 4 4 |

So the intermediate "fix the IN channel" commits
(`73088bd36`, `1ea1d565d`, `91fd7088c`, `9cc3abeda`, `9e4ec0c4d`) chased the
host-side XOFF artifact and are **not functionally required**. The serialized
pump they ended up with is still the right design (never re-arm over a live
transfer), so the code stays — the history was squashed instead, so that no
commit claims to fix a bug that never existed.

Only three things actually mattered: the DCD-based stack (`ec07e2172`, the WCH
driver really could not transmit), the status/query frames themselves, and the
raw-mode host tools.

## Reproduce

```bash
cd examples/dual/hid_cdc_bridge
make BOARD=nanoch32v203 all && wlink flash _build/nanoch32v203/hid_cdc_bridge.elf && wlink reset

# raw-mode reader (the tools do this themselves)
stty -F /dev/ttyACM1 raw -crtscts -ixon
./tools/cdc_tui.py --selftest          # 4 checks
./tools/bridge_check.py /dev/ttyACM1   # 25 checks (needs the HID cable on this PC)

# query round trip: TUI key 't', or
python3 - <<'PY'
import tty, termios, os, struct, time, select
fd = os.open("/dev/ttyACM1", os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd); a = termios.tcgetattr(fd); a[2] &= ~termios.CRTSCTS
a[0] &= ~(termios.IXON | termios.IXOFF); termios.tcsetattr(fd, termios.TCSANOW, a)
os.set_blocking(fd, False)
F = struct.Struct("<H8sH")
os.write(fd, F.pack(0x8758, bytes([7]) + bytes(7), 0x0900))
end = time.time() + 1
while time.time() < end:
    r, _, _ = select.select([fd], [], [], 0.05)
    if r: print(os.read(fd, 256).hex(" "))
os.close(fd)
PY
```

## Files

| Path | Role |
|---|---|
| `src/usbfs_dcd.c` | USBFS controller driver (copy of TinyUSB's, events redirected) |
| `src/usbfs_mini.h` | DCD prototypes + event entry points |
| `src/usbfs_cdc.c` | mini device stack, CDC-ACM, rings, serialized TX pump + safety valve |
| `src/bridge_protocol.[ch]` | 12-byte frames: input frames, `0x8757` status, `0x8758` query |
| `src/main.c` | bridge policy, HID reports, status on change or query |
| `tools/cdc_tui.py` | interactive tool (`--selftest` headless), raw mode |
| `tools/bridge_check.py` | 25-check functional suite, raw mode |
