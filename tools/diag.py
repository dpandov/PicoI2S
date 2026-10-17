"""Read the PicoI2S diagnostic string from a running device.

The firmware serves a hidden USB string descriptor (index 0x20) with the
previous-boot crash record and live status. It is read through the
picotool reset interface, which Windows binds to WinUSB automatically.

    pip install pyusb libusb-package
    python tools/diag.py            # one reading
    python tools/diag.py 10         # ten readings, one per second

Output fields:
    b<N>        boot count since power-up or the last flash
    none/...    how the previous boot ended: none, panic, fault, hang, wdog
    m<N>        (after a crash) last breadcrumb, see DIAG_MARK_* in src/diag/diag.h
    pc / lr     (after a crash) code address of the panic message, fault or hang
    |           everything after the last "| " is the application status:
    D/I/S       link state: detached, idle, streaming
    a/o/1       UAC2 mode: host detected as Apple (3-byte 10.14 feedback) or
                other; 1: UAC1 mode (mode pin GP5 grounded)
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
        tone+selftest> mode=<uac2|uac1> format=<i2s|lj|rj16|rj24|tda1541a|
        dual16|dual18|dual20|dual24> filter=<nos|sharp|slow|sinc|minphase>
        jumpers=<GP5>/<GP8 GP7 GP6>/<GP2 GP3 GP4> clocks=<stop|keep>

    filter is what the filter jumpers select (reserved codes: sharp), also
    for I2S / LJ / RJ, which never use it. jumpers: 1 = grounded. clocks:
    keep with the PICOI2S_KEEP_CLOCKS build, whose I2S / dual-line clocks
    keep running with silence (S/PDIF still stops; tx follows S/PDIF).
    e.g. board=rp2350-one fw=0104 build=normal mode=uac2 format=i2s filter=nos jumpers=0/000/000 clocks=stop

A fourth hidden string (index 0x25) identifies the firmware build:

    ver=<major.minor> build=<N> git=<commit> dirty=<0|1> date=<UTC time>
        sdk=<Pico SDK version> tinyusb=<TinyUSB version>

    ver is the version in CMakeLists.txt (also fw, the USB bcdDevice);
    build is the number of commits in HEAD, git the commit, dirty 1 when
    the tracked files differed from it at build time; date the build time
    (UTC). Without git: build=0 git=unknown.
    e.g. ver=1.04 build=20 git=ca5ef88 dirty=0 date=2026-10-11T08:13Z sdk=2.3.1 tinyusb=0.21.0

A fifth hidden string (index 0x26) carries the output state, at most 81
characters:

    clk<Hz> pll<+|-><ppm>|pll- i2s<osr>|i2s- w<bits>|w- e<0|1>
        rxl<frames>|rxl- urt<N> ovt<N> bad<N>

    clk         clk_sys in Hz (set per sample rate, see the README's clock
                table)
    pll         rate error of the running output at zero trim against its
                nominal rate in ppm (the clock table's), pll- when stopped
    i2s         I2S / dual-line output frames per sample: 8 or 4 with an
                oversampling filter, 1 otherwise; i2s- when not running
    w           S/PDIF word length sent: 16, or 24 (24-bit streams, the
                volume below 0 dB, the input's channel status); w- stopped
    e           1 while pre-emphasised input samples go out (S/PDIF flag
                set, the I2S path de-emphasising)
    rxl         S/PDIF receiver FIFO level in frames while an input is
                selected (held at 6 ms while it plays), rxl- otherwise
    urt, ovt    underruns and overruns since boot
    bad         USB packets with no data or not whole stereo frames, since
                boot
    Totals stop at 999999.
    e.g. clk147428571 pll-186 i2s8 w24 e0 rxl- urt0 ovt0 bad0
"""
import re
import sys
import time

import libusb_package
import usb.core
import usb.util

VID = 0x1209
# Both USB modes share the PID; firmware 1.04 and earlier used a second one
# in UAC1 mode.
PID = 0x0001
PIDS = (PID, 0x0002)
STRID_DIAG = 0x20
STRID_SESSION = 0x23
STRID_CONFIG = 0x24
STRID_VERSION = 0x25
STRID_OUTPUT = 0x26
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


def parse_version(s):
    """Version string -> dict of its key=value fields, or None (no device,
    or firmware without the string)."""
    fields = dict(t.split("=", 1) for t in s.split() if "=" in t)
    return fields if "ver" in fields and "build" in fields else None


def parse_output(s):
    """Output state string -> dict, or None if it is not one.

    Keys: clk_hz, pll_ppm (None when stopped), osr (None when the I2S
    output is not running), word_bits (None when stopped), emphasis,
    rx_level (None without a selected input), underruns, overruns,
    bad_packets.
    """
    m = re.fullmatch(r"clk(\d+) pll([+-]\d+|-) i2s(\d+|-) w(\d+|-) e([01]) rxl(\d+|-) "
                     r"urt(\d+) ovt(\d+) bad(\d+)", s.strip())
    if not m:
        return None
    opt = [None if g == "-" else int(g) for g in m.group(2, 3, 4, 6)]
    return {"clk_hz": int(m.group(1)), "pll_ppm": opt[0], "osr": opt[1], "word_bits": opt[2],
            "emphasis": m.group(5) == "1", "rx_level": opt[3], "underruns": int(m.group(7)),
            "overruns": int(m.group(8)), "bad_packets": int(m.group(9))}


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    backend = libusb_package.get_libusb1_backend()
    version = read_diag(backend, STRID_VERSION)
    if parse_version(version):
        print("version:", version)
    config = read_diag(backend, STRID_CONFIG)
    if parse_config(config):
        print("config:", config)
    for i in range(count):
        print(time.strftime("%H:%M:%S"), read_diag(backend))
        session = read_diag(backend, STRID_SESSION)
        if parse_session(session):
            print("         session:", session)
        output = read_diag(backend, STRID_OUTPUT)
        if parse_output(output):
            print("         output: ", output)
        if i + 1 < count:
            time.sleep(1)


if __name__ == "__main__":
    main()
