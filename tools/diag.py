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
    run0/1      USB stream samples going out on S/PDIF (the S/PDIF input
                playing shows in the session string: src)
    led0/1      onboard LED state (any colour)
    tx<Hz>      S/PDIF output sample rate, 0 when the output is stopped
    ov<N>       overruns: USB packets that overwrote unread FIFO data
    lt<N>       S/PDIF interrupts too late to refill a buffer in time
    rmin<N>     lowest output ring level while playing, in frames (target 192)
    fmax<N>     highest receive FIFO level in bytes
    lp<us>      longest main-loop iteration in microseconds
                (~1000 at 96 kHz loses a USB packet)
    tr<ppm>L/U  while the output follows an external clock (UAC1 mode, or
                the S/PDIF input playing): rate trim in ppm, +186 (32/48 kHz)
                or +344 (44.1 kHz) to cancel the PLL error, plus the source
                clock's offset against the Pico crystal. L = clock servo
                locked, U = acquiring or not playing
    v<dB>/vM    host volume (Feature Unit): attenuation in dB, 0 to 64, or M
                when muted (or set below -64 dB). v0 is bit-perfect; below
                0 dB the output is scaled and flagged as 24-bit. Last token
                in both modes
    ur, ov, lt and rmin count from the start of the current or last stream
    (open or format change); fmax and lp from when it began playing, so the
    start-up (pre-roll, clock switch) is excluded. Counters stop at 999999
    (9999 while the trim is shown).

    e.g. UAC2:  So 48000/24 ff4712 ur0 run1 led1 tx48000 ov0 lt0 rmin144 fmax5240 lp212 v0
         UAC1:  S1 44100/16 ff4935 ur0 run1 led1 tx44100 ov0 lt0 rmin144 fmax5400 lp212 tr361L v12

A second hidden string (index 0x23, read by tools/monitor.py and printed
by this script after each reading) carries the session counters since boot
and the S/PDIF input state, at most 123 characters:

    n<streams> fr<frames> sig<ms|-> up<ms> os<late> src<u|c|t|->
        in<c|t|->/<c|-><t|-> ir<rate><+|-><ppm>|ir- cs<bits>[e][n]|cs-
        rx<locks>/<unlocks>/<parity>/<framing>/<overflows>

    n           USB stream starts (open, or format change while open)
    fr          frames moved to the output, from either source (wraps at 2^32)
    sig         milliseconds since the last non-zero sample went out (- if
                none yet)
    up          uptime in milliseconds
    os          I2S blocks that missed their DMA buffer (i2s_tx_late())
    src         source heard: u USB, c coax input, t Toslink input, - none
                (pre-roll, underrun, stall, output off)
    in          S/PDIF input selected by the receiver (c, t, or - when
                neither is locked), then each input's own lock: c / t, or -
    ir          nominal rate of the selected input and its measured offset
                in ppm, or ir- without a selected input
    cs          channel status of the selected input: word length, e when
                50/15 us pre-emphasis is flagged, n for non-PCM data (played
                as zeros); cs- until a whole block has arrived
    rx          receiver counters since boot: lock acquisitions, lock losses,
                parity errors, preamble / framing errors, FIFO overflows

    Counters from os on stop at 99999, ppm at +/-9999. Older firmware ends
    after up (or os); the missing fields parse as None.
    e.g. n3 fr91234567 sig12 up734002 os0 srcc inc/ct ir44100+12 cs24e rx3/1/0/2/0

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
    rate, bits, volume (dB or 'M'), trim / locked (when trimmed), fields (the
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


_SOURCES = {"u": "usb", "c": "coax", "t": "toslink", "-": "none"}
_INPUTS = {"c": "coax", "t": "toslink", "-": "none"}


def parse_session(s):
    """Session string -> dict, or None if it is not one.

    Keys: streams, frames, signal_age_ms (None: no signal yet), up_ms,
    i2s_late; source ('usb', 'coax', 'toslink', 'none'), input (selected:
    'coax', 'toslink', 'none'), locked ({'coax': bool, 'toslink': bool}),
    input_rate (0 if none), input_ppm, cs ({'bits', 'emphasis', 'pcm'}, or
    {} while unknown), rx ({'locks', 'unlocks', 'parity', 'framing',
    'overflows'}). A key whose token the firmware does not send (older
    versions) is None.
    """
    m = re.match(r"n(\d+) fr(\d+) sig(\d+|-) up(\d+)(?= |$)", s.strip())
    if not m:
        return None
    d = {"streams": int(m.group(1)), "frames": int(m.group(2)),
         "signal_age_ms": None if m.group(3) == "-" else int(m.group(3)),
         "up_ms": int(m.group(4)), "i2s_late": None, "source": None, "input": None,
         "locked": None, "input_rate": None, "input_ppm": None, "cs": None, "rx": None}
    for tok in s.split()[4:]:
        if m := re.fullmatch(r"os(\d+)", tok):
            d["i2s_late"] = int(m.group(1))
        elif m := re.fullmatch(r"src([uct-])", tok):
            d["source"] = _SOURCES[m.group(1)]
        elif m := re.fullmatch(r"in([ct-])/([c-])([t-])", tok):
            d["input"] = _INPUTS[m.group(1)]
            d["locked"] = {"coax": m.group(2) == "c", "toslink": m.group(3) == "t"}
        elif tok == "ir-":
            d["input_rate"], d["input_ppm"] = 0, None
        elif m := re.fullmatch(r"ir(\d+)([+-]\d+)", tok):
            d["input_rate"], d["input_ppm"] = int(m.group(1)), int(m.group(2))
        elif tok == "cs-":
            d["cs"] = {}
        elif m := re.fullmatch(r"cs(\d+)(e?)(n?)", tok):
            d["cs"] = {"bits": int(m.group(1)), "emphasis": m.group(2) == "e",
                       "pcm": m.group(3) != "n"}
        elif m := re.fullmatch(r"rx(\d+)/(\d+)/(\d+)/(\d+)/(\d+)", tok):
            d["rx"] = dict(zip(("locks", "unlocks", "parity", "framing", "overflows"),
                               map(int, m.groups())))
    return d


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
        session = read_diag(backend, STRID_SESSION)
        if parse_session(session):
            print("         session:", session)
        if i + 1 < count:
            time.sleep(1)


if __name__ == "__main__":
    main()
