"""Read the usb2spdif diagnostic string from a running device.

The firmware serves a hidden USB string descriptor (index 0x20) with the
previous-boot crash record and live status. It is read through the
picotool reset interface, which Windows binds to WinUSB automatically.

    pip install pyusb libusb-package
    python tools/diag.py            # one reading
    python tools/diag.py 10         # ten readings, one per second

Output fields:
    b<N>        boot count since power-up
    none/...    how the previous boot ended: none, panic, fault, hang, wdog
    m<N>        (after a crash) last breadcrumb, see DIAG_MARK_* in src/diag/diag.h
    pc / lr     (after a crash) code address of the panic message, fault or hang
    |           everything after the last "| " is the application status:
    D/I/S       link state: detached, idle, streaming
    a/o/1       UAC2 mode: host detected as Apple (3-byte 10.14 feedback) or
                other; 1: UAC1 mode (mode pin GP5 grounded, PID 0x0002)
    rate/bits   current stream format
    ff<N>       receive FIFO level in bytes (depth 9312, held at 6 packets)
    ur<N>       underruns: output ring ran dry while playing
    run0/1      stream samples going out on S/PDIF
    led0/1      onboard LED state
    tx<Hz>      S/PDIF output sample rate, 0 when the output is stopped
    ov<N>       overruns: USB packets that overwrote unread FIFO data
    lt<N>       S/PDIF interrupts too late to refill a buffer in time
    rmin<N>     lowest output ring level while playing, in frames (target 192)
    fmax<N>     highest receive FIFO level in bytes
    lp<us>      longest main-loop iteration in microseconds
                (~1000 at 96 kHz loses a USB packet)
    tr<ppm>L/U  UAC1 mode only: S/PDIF rate trim in ppm, the adaptive clock
                following the host: +186 (32/48 kHz) or +344 (44.1 kHz) to
                cancel the PLL error, plus the host clock offset against the
                Pico crystal. L = clock servo locked, U = acquiring or not
                playing
    v<dB>/vM    host volume (Feature Unit): attenuation in dB, 0 to 64, or M
                when muted (or set below -64 dB). v0 is bit-perfect; below
                0 dB the output is scaled and flagged as 24-bit. Last token
                in both modes
    ur, ov, lt and rmin count from the start of the current or last stream
    (open or format change); fmax and lp from when it began playing, so the
    start-up (pre-roll, clock switch) is excluded. Counters stop at 999999
    (9999 in UAC1 mode).

    e.g. UAC2:  So 48000/24 ff4712 ur0 run1 led1 tx48000 ov0 lt0 rmin144 fmax5240 lp212 v0
         UAC1:  S1 44100/16 ff4935 ur0 run1 led1 tx44100 ov0 lt0 rmin144 fmax5400 lp212 tr361L v12

A second hidden string (index 0x23, read by tools/monitor.py) carries the
session counters since boot: "n<streams> fr<frames> sig<ms|-> up<ms> os<late>":
stream starts, frames moved to the output (wraps at 2^32), milliseconds
since the last non-zero sample went out (- if none yet), uptime, and I2S
blocks that missed their DMA buffer (i2s_tx_late(); absent in older
firmware).

A third hidden string (index 0x24, printed by this script and by
tools/monitor.py) carries the configuration, fixed at boot:

    board=<rp2350-one|pico2> fw=<bcdDevice hex> build=<normal|selftest|tone|
        tone+selftest> mode=<uac2|uac1> format=<i2s|lj|rj24|rj16|dual16|
        dual18|dual20|dual24> filter=<nos|sharp|slow|sinc|minphase>
        jumpers=<GP5>/<GP8 GP7 GP6>/<GP2 GP3 GP4>

    filter is what the filter jumpers select (reserved codes: sharp), also
    for I2S / LJ / RJ, which never use it. jumpers: 1 = grounded.
    e.g. board=rp2350-one fw=0103 build=normal mode=uac2 format=i2s filter=nos jumpers=0/000/000
"""
import re
import sys
import time

import libusb_package
import usb.core
import usb.util

VID = 0x1209
PIDS = {0x0001: "UAC2", 0x0002: "UAC1"}
STRID_DIAG = 0x20
STRID_SESSION = 0x23
STRID_CONFIG = 0x24
LANGID_EN_US = 0x0409

_INT_TOKEN = re.compile(r"^([a-z]+)(\d+)$")


# Two tools polling at once (monitor.py beside selftest.py) collide on the
# WinUSB handle: access denied or busy. Those are retried briefly.
_RETRY_ERRNOS = (13, 16)
_RETRIES = 5
_RETRY_DELAY_S = 0.02


def read_diag(backend, index=STRID_DIAG):
    for attempt in range(_RETRIES):
        dev = usb.core.find(idVendor=VID, backend=backend,
                            custom_match=lambda d: d.idProduct in PIDS)
        if dev is None:
            return "<device not found>"
        try:
            return usb.util.get_string(dev, index, LANGID_EN_US)
        except usb.core.USBError as e:
            if e.errno not in _RETRY_ERRNOS or attempt == _RETRIES - 1:
                return f"<error: {e}>"
            time.sleep(_RETRY_DELAY_S)
        finally:
            usb.util.dispose_resources(dev)


def parse_status(s):
    """Diagnostic string -> dict, or None if it is not one.

    Keys: boots, prev, link ('D', 'I', 'S'), host ('a', 'o', '1'), uac1,
    rate, bits, volume (dB or 'M'), trim / locked (UAC1), fields (the
    integer tokens: ff, ur, run, led, tx, ov, lt, rmin, fmax, lp).
    """
    head, sep, app = s.partition(" | ")
    m = re.match(r"^b(\d+) (\S+)", head)
    if not m or not sep:
        return None
    d = {"raw": s, "boots": int(m.group(1)), "prev": m.group(2), "fields": {},
         "link": None, "host": None, "uac1": False, "rate": None, "bits": None,
         "volume": None, "trim": None, "locked": None}
    tokens = app.split()
    if tokens and re.fullmatch(r"[DIS][ao1]", tokens[0]):
        d["link"], d["host"] = tokens[0][0], tokens[0][1]
        d["uac1"] = d["host"] == "1"
        tokens = tokens[1:]
    for tok in tokens:
        m = re.fullmatch(r"tr(-?\d+)([LU])", tok)
        if m:
            d["trim"], d["locked"] = int(m.group(1)), m.group(2) == "L"
            continue
        m = re.fullmatch(r"v(\d+|M)", tok)
        if m:
            d["volume"] = "M" if m.group(1) == "M" else int(m.group(1))
            continue
        m = re.fullmatch(r"(\d+)/(\d+)", tok)
        if m:
            d["rate"], d["bits"] = int(m.group(1)), int(m.group(2))
            continue
        m = _INT_TOKEN.match(tok)
        if m:
            d["fields"][m.group(1)] = int(m.group(2))
    return d


def parse_session(s):
    """Session string -> dict (streams, frames, signal_age_ms or None, up_ms,
    i2s_late or None without the token)."""
    m = re.fullmatch(r"n(\d+) fr(\d+) sig(\d+|-) up(\d+)( os(\d+))?", s.strip())
    if not m:
        return None
    return {"streams": int(m.group(1)), "frames": int(m.group(2)),
            "signal_age_ms": None if m.group(3) == "-" else int(m.group(3)),
            "up_ms": int(m.group(4)),
            "i2s_late": int(m.group(6)) if m.group(6) is not None else None}


def parse_config(s):
    """Configuration string -> dict of its key=value fields, or None (no
    device, or firmware without the string)."""
    fields = dict(t.split("=", 1) for t in s.split() if "=" in t)
    return fields if "board" in fields and "format" in fields else None


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    backend = libusb_package.get_libusb1_backend()
    config = read_diag(backend, STRID_CONFIG)
    if parse_config(config):
        print("config:", config)
    for i in range(count):
        print(time.strftime("%H:%M:%S"), read_diag(backend))
        if i + 1 < count:
            time.sleep(1)


if __name__ == "__main__":
    main()
