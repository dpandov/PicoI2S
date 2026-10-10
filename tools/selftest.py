"""usb2spdif self-test: bit-exact check of the S/PDIF and I2S outputs
(Windows, Linux).

Needs firmware built with -DUSB2SPDIF_SELFTEST=ON. The Pico then decodes
its own S/PDIF pin and its I2S / dual-line pins on core 1 and reports the
results as USB strings 0x21 and 0x22; no receiver, probe or UART is needed.
This script plays a test pattern and reads the reports. Windows: a native
WASAPI exclusive-mode player (tools/wasapi_play.c, built on first use with
the ziglang package into tools/build/). Linux: aplay (alsa-utils) on the
bridge's ALSA hw device, fed by this script; hw has no format conversion
and no software volume, so it is bit-exact (stop PipeWire / PulseAudio
from holding the card first). Reading the strings needs access to the
USB device (udev rule, see tools/README.md).

    pip install -r tools/requirements.txt
    python tools/selftest.py                     sweep: 5 rates x 16/24 bit, 3-s stop
    python tools/selftest.py --only 44100/16     one format
    python tools/selftest.py --soak 60 --rate 96000 --bits 24
                                                 one format for 60 minutes
    python tools/selftest.py --status            print the three reports once
    python tools/selftest.py --i2s-format dual20 require this I2S format (jumpers)
    python tools/selftest.py --i2s-filter slow   require this dual-line filter (jumpers)

Sweep order: 48k/16, 48k/24, 44.1k/24, 96k/16, 88.2k/16, 88.2k/24, 32k/24,
32k/16, 44.1k/16, 96k/24: 16 <-> 24 bit at the same rate and clock family
switches in both directions. In UAC1 mode (mode pin grounded, detected from
the diagnostic string) the device only offers 16 bit at up to 48 kHz: the
sweep is 48k/16, 44.1k/16, 32k/16 (clock family switches both ways). There
the output clock follows the host (adaptive), so the measured rate is the
host's clock against the Pico crystal; the same +-2000 ppm check applies,
and the S/PDIF rate trim and clock servo lock are reported. A trim at the
servo limit (+-1500 ppm) fails. Each format plays for 7 s and is read at
2.5 ... 6.5 s. It passes when every read shows the stream and S/PDIF output
at the requested format, a clean window of pattern frames only ("ok": no
silent frames, no cell / parity / order / V/U / pattern errors), the right
channel status (rate and word length codes), a measured frame rate within
+-2000 ppm, fresh results, and no reboot; and when, from the first to the
last read, no bad, gap or skipped windows were added and the device's
underrun / overrun / late-interrupt counters (ur, ov, lt) did not increase.
After the sweep the output must send silence 2 s after the stream closed and
be stopped (line idle low) 4.5 s after.

The I2S / dual-line output runs in lockstep with the S/PDIF one in the
format selected by the format jumpers and, for the dual-line formats, with
the filter selected by the filter jumpers; the device reports both by
name. Every row of the sweep gets an I2S result too: PASS when every read
shows the I2S window "ok" at the requested stream format (frames, no
silent frames, no sample errors, no framing errors) in the expected format
(--i2s-format, default: the format the device reports at the start) with
the expected filter (--i2s-filter, default: the filter the device reports
at the start) at its oversampling for the rate (sharp, slow, minphase: 8
below 50 kHz, 4 above), and no bad, gap or skipped windows and no late
filter blocks (L) were added. The direct formats (i2s, lj, rj16 or rj24)
and the dual-line formats with the nos filter are checked against the
pattern (a format narrower than the stream, rj16, tda1541a or dual16 ..
dual20 at 24 bit, at its own width); the filtered dual-line formats (sharp, slow, minphase, and
sinc at the sample rate) are checked bit-exact against the window's own
S/PDIF samples run through the transmitter's filter on the device, so no
host-side model is involved. The sweep prints the filter token (sharp8,
slow4, sinc, nos, - ...) in the I2S column. The stop test expects the I2S
output silent, then stopped with all pins low. The soak logs every I2S
failure and counter increase as an event.

The device has a volume control that Windows applies even in exclusive
mode, so the script first sets the Windows volume of the device to 100 %
and unmutes it (wasapi_play --set-volume-max; a failure is only a warning),
and every read fails while the device reports an attenuation or mute
(diagnostic token v<dB> other than v0, or vM).

Soak: plays one format continuously and reads every few seconds; any failed
check or counter increase is logged with a timestamp. Passes only with zero
events. Bad windows are counted on the device, so a glitch between two
reads is still caught.

With a test-tone build (-DUSB2SPDIF_TEST_TONE=ON as well) the output is not
the pattern (report shows "s-"): the script then checks framing, channel
status and rate of the tone only, plus the I2S framing; a filtered
dual-line output still gets its bit-exact check against the window's
S/PDIF samples run through the filter (its "s" stays numeric and must be
0).

Self-test report (see src/selftest/selftest_report.h):

    #<seq> <state> tx<rate>/<bits> f<frames> z<silent> e<cell> p<parity>
        o<order> v<V/U> s<pattern|-> c<match><cs3><cs4> fs<Hz>
        B<bad> G<gaps> K<skipped> [@<frame>:<prev L>><L>/<R>]

    state   ok, sil (silent frames), idle (output stopped, line static),
            wait; failures in capitals: ERR, NOSYNC, NOCARR, CARRIER,
            CAPTMO, NOPIO
    c       1 channel status matches, 0 differs, - no complete block,
            x C bit differs between subframes; cs3 rate code (00 44.1k,
            02 48k, 03 32k, 08 88.2k, 0A 96k), cs4 length (02 16, 0B 24)
    B G K   cumulative windows: bad, with gaps (dropouts), skipped
            (format or clock change during the capture)
    e.g.    #812 ok tx48000/24 f577 z0 e0 p0 o0 v0 s0 c1020B fs47991 B2 G0 K1

I2S report (same header, USB string 0x22, same seq as the S/PDIF window):

    #<seq> <state> <fmt> <flt> tx<rate>/<bits> f<frames> z<silent> s<errors|->
        e<framing> L<late> B<bad> G<gaps> K<skipped>
        [@<frame>:<prev or expected L>><L>/<R>]

    fmt     jumper format: i2s, lj, rj16 (rj24 with the USB2SPDIF_RJ24 build),
            tda1541a, dual16, dual18, dual20, dual24
    flt     dual-line filter (jumpers): sharp, slow or minphase with the
            oversampling of the running output appended (sharp8, slow4, ...;
            the bare name while stopped), sinc, nos; - for the direct formats
    state   ok, sil, idle (stopped, pins low), wait; failures in capitals:
            ERR, NOSYNC (LRCK static / no LATCH), NOCLK (no BCLK edges),
            NOTIM (no timing for the format at this rate), OFF (S/PDIF
            runs, I2S does not), PINHI (stopped, a pin high: lvl<hex> =
            DATA, DR, BCLK, LRCK in bits 0..3), NOPIO
    s       pattern violations (direct, nos) or frames differing from the
            filtered S/PDIF reference (sharp, slow, minphase, sinc); "-"
            when no sample check ran
    L       cumulative blocks that missed the I2S output
    e.g.    #812 ok i2s - tx48000/24 f320 z0 s0 e0 L0 B0 G0 K1
            #812 ok dual16 sharp8 tx48000/24 f1280 z0 s0 e0 L0 B0 G0 K1

The diagnostic string (tools/diag.py) supplies the boot count and the
application status; its name<number> tokens are parsed generically.
"""
import argparse
import os
import re
import subprocess
import sys
import threading
import time

try:
    import libusb_package
except ImportError:
    sys.exit("missing Python packages: pip install -r tools/requirements.txt")

from diag import STRID_DIAG, read_diag

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PLAYER_SRC = os.path.join(HERE, "wasapi_play.c")
PLAYER_DEPS = [PLAYER_SRC, os.path.join(REPO, "src", "selftest", "selftest_pattern.h")]
PLAYER_EXE = os.path.join(HERE, "build", "wasapi_play.exe")

STRID_SELFTEST = 0x21
STRID_SELFTEST_I2S = 0x22
I2S_FORMATS = ("i2s", "lj", "rj16", "rj24", "tda1541a", "dual16", "dual18", "dual20", "dual24")
DUAL_FORMATS = ("tda1541a", "dual16", "dual18", "dual20", "dual24")
I2S_FILTERS = ("sharp", "nos", "slow", "minphase", "sinc")
I2S_OVERSAMPLING_FILTERS = ("sharp", "slow", "minphase")
# Filter token: an oversampling filter with the running output's
# oversampling (bare while stopped), sinc, nos, or - (direct formats).
_FILTER_TOKEN = re.compile(r"^(?:(sharp|slow|minphase)([48])?|(sinc|nos|-))$")

SWEEP = [(48000, 16), (48000, 24), (44100, 24), (96000, 16), (88200, 16),
         (88200, 24), (32000, 24), (32000, 16), (44100, 16), (96000, 24)]
PLAY_SECONDS = 7.0
SAMPLE_AT = (2.5, 3.5, 4.5, 5.5, 6.5)     # the first read is the baseline
SWEEP_UAC1 = [(48000, 16), (44100, 16), (32000, 16)]
UAC1_RATES = (32000, 44100, 48000)
TRIM_LIMIT_PPM = 1500   # CLOCK_SERVO_MAX_PPB
RATE_CODE = {32000: 0x03, 44100: 0x00, 48000: 0x02, 88200: 0x08, 96000: 0x0A}
LEN_CODE = {16: 0x02, 24: 0x0B}
MAX_PPM = 2000
STOP_SILENCE_AT = 2.0   # s after the stream closed: still sending silence
STOP_DONE_AT = 4.5      # s after the stream closed: output stopped
DEVICE_COUNTERS = ("ur", "ov", "lt")      # must not increase while playing
WINDOW_COUNTERS = ("B", "G", "K")
I2S_COUNTERS = WINDOW_COUNTERS + ("L",)   # plus late filter blocks
SOAK_WARMUP = 3.0
SOAK_MARGIN = 2.0       # pattern keeps playing this long after the last read
TONE_RATE, TONE_BITS = 48000, 24


#--------------------------------------------------------------------------
# Player
#--------------------------------------------------------------------------

class PlayerBuildError(Exception):
    pass


def build_player():
    """Builds tools/build/wasapi_play.exe if missing or out of date."""
    if os.path.exists(PLAYER_EXE) and all(
            os.path.getmtime(PLAYER_EXE) >= os.path.getmtime(d) for d in PLAYER_DEPS):
        return PLAYER_EXE
    try:
        import ziglang  # noqa: F401
    except ImportError:
        raise PlayerBuildError("building wasapi_play needs the ziglang package: "
                               "pip install -r tools/requirements.txt")
    os.makedirs(os.path.dirname(PLAYER_EXE), exist_ok=True)
    print("building tools/build/wasapi_play.exe ...")
    cmd = [sys.executable, "-m", "ziglang", "cc", "-target", "x86_64-windows-gnu", "-O2",
           "-std=c11", "-o", PLAYER_EXE, PLAYER_SRC, "-lole32"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise PlayerBuildError(f"wasapi_play build failed:\n{r.stdout}{r.stderr}")
    return PLAYER_EXE


def set_volume_max(exe=None, device=None):
    """Sets the device's Windows volume to 100 % and unmutes it. The device
    has a volume control, which Windows applies in exclusive mode too, so
    anything else is not bit-exact. Warns and returns False on failure."""
    try:
        exe = exe or build_player()
        r = subprocess.run([exe, "--set-volume-max"] + ([device] if device else []),
                           capture_output=True, text=True, timeout=10)
    except (PlayerBuildError, OSError, subprocess.SubprocessError) as e:
        print(f"warning: could not set the device volume to 100 %: {e}")
        return False
    if r.returncode != 0:
        print(f"warning: could not set the device volume to 100 % (exit {r.returncode}): "
              f"{(r.stderr or r.stdout).strip()}")
        return False
    print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "volume: set")
    return True


class Player:
    """wasapi_play in the background: pattern for `seconds`, then silence."""

    def __init__(self, exe, rate, bits, seconds, device=None):
        cmd = [exe, str(rate), str(bits), f"{seconds:g}"] + ([device] if device else [])
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True)

    def wait(self):
        """Exit code and error text (empty on success)."""
        out, err = self.proc.communicate()
        return self.proc.returncode, (err or out).strip()

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.communicate()


# Linux: the bridge's ALSA card and aplay.

TRAILING_SILENCE = 0.2  # s of zeros after the pattern, so the stream ends cleanly
BRIDGE_NAMES = ("USB to S/PDIF", "USB to SPDIF")


def alsa_card(device=None):
    """Index of the first ALSA card whose /proc/asound/cards line contains
    `device` (default: the bridge's name in either mode), or None."""
    names = (device,) if device else BRIDGE_NAMES
    try:
        with open("/proc/asound/cards") as f:
            lines = f.read().splitlines()
    except OSError:
        return None
    for line in lines:
        m = re.match(r"^\s*(\d+) \[", line)
        if m and any(n in line for n in names):
            return int(m.group(1))
    return None


def alsa_volume_max(card):
    """Sets every playback control of the card to its maximum (0 dB) and
    unmutes it: the device applies its Feature Unit volume itself. Warns
    and returns False on failure."""
    try:
        r = subprocess.run(["amixer", "-c", str(card), "scontrols"], capture_output=True,
                           text=True, timeout=10)
        controls = re.findall(r"^Simple mixer control '(.*)',(\d+)$", r.stdout, re.M)
        for name, idx in controls:
            subprocess.run(["amixer", "-q", "-c", str(card), "sset", f"{name},{idx}", "100%",
                            "unmute"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as e:
        print(f"warning: could not set the device volume to 0 dB: {e}")
        return False
    if not controls:
        print("warning: no mixer control found on the card")
        return False
    print(f"volume: {', '.join(n for n, _ in controls)} set to 0 dB, unmuted")
    return True


def pattern_bytes(start, n, bits):
    """n frames of the self-test pattern (src/selftest/selftest_pattern.h)
    from frame `start`, interleaved little-endian as the stream carries
    them: int16 at 16 bits, 24 bits in the top of 32-bit containers."""
    import numpy as np
    k = np.arange(start, start + n, dtype=np.uint64)
    out = np.empty((n, 2), dtype=np.uint32)
    if bits == 16:
        left = (k * 0x5A37) & 0xFFFF
        out[:, 0] = left
        out[:, 1] = ~left & 0xFFFF
        return out.astype("<u2").tobytes()
    left = (k * 0x0B5A37) & 0xFFFFFF
    out[:, 0] = left << 8
    out[:, 1] = (~left & 0xFFFFFF) << 8
    return out.astype("<u4").tobytes()


class AlsaPlayer:
    """aplay on hw:<card>,0 in the background: pattern for `seconds`, then
    a little silence. Same interface as Player."""

    def __init__(self, card, rate, bits, seconds):
        fmt = "S16_LE" if bits == 16 else "S32_LE"
        cmd = ["aplay", "-q", "-D", f"hw:{card},0", "-t", "raw", "-f", fmt, "-r", str(rate),
               "-c", "2"]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.PIPE)
        self.stop = False
        self.thread = threading.Thread(target=self._feed, args=(rate, bits, seconds),
                                       daemon=True)
        self.thread.start()

    def _feed(self, rate, bits, seconds):
        total = int(seconds * rate + 0.5)
        chunk = max(rate // 20, 1)
        sent = 0
        try:
            while sent < total and not self.stop:
                n = min(chunk, total - sent)
                self.proc.stdin.write(pattern_bytes(sent, n, bits))
                sent += n
            sample_bytes = 2 if bits == 16 else 4
            self.proc.stdin.write(bytes(int(TRAILING_SILENCE * rate) * 2 * sample_bytes))
            self.proc.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass

    def wait(self):
        """Exit code and error text (empty on success)."""
        self.thread.join()
        err = self.proc.stderr.read()
        self.proc.wait()
        return self.proc.returncode, err.decode(errors="replace").strip()

    def kill(self):
        self.stop = True
        if self.proc.poll() is None:
            self.proc.kill()
        self.thread.join()
        self.proc.stderr.read()
        self.proc.wait()


def make_player(exe, rate, bits, seconds, device=None):
    """The platform's player; exe is the WASAPI player on Windows, the ALSA
    card index on Linux."""
    if sys.platform == "win32":
        return Player(exe, rate, bits, seconds, device)
    return AlsaPlayer(exe, rate, bits, seconds)


#--------------------------------------------------------------------------
# Reports
#--------------------------------------------------------------------------

_INT_TOKEN = re.compile(r"^([A-Za-z]+)(\d+)(?:/\d+)?$")


def parse_report(s):
    """Self-test report -> dict, None if s is not one."""
    t = s.split()
    if len(t) < 2 or not re.fullmatch(r"#\d+", t[0]):
        return None
    d = {"raw": s, "seq": int(t[0][1:]), "state": t[1], "pattern": True}
    for tok in t[2:]:
        m = re.fullmatch(r"tx(\d+)(?:/(\d+))?", tok)
        if m:
            d["tx"] = (int(m.group(1)), int(m.group(2) or 0))
            continue
        m = re.fullmatch(r"c([01x-])([0-9A-F]{2})([0-9A-F]{2})", tok)
        if m:
            d["csflag"], d["cs3"], d["cs4"] = m.group(1), int(m.group(2), 16), int(m.group(3), 16)
            continue
        m = re.fullmatch(r"@(\d+):([0-9A-F]{6})>([0-9A-F]{6})/([0-9A-F]{6})", tok)
        if m:
            d["viol"] = (int(m.group(1)), m.group(2), m.group(3), m.group(4))
            continue
        if tok == "s-":
            d["pattern"] = False
            continue
        m = _INT_TOKEN.match(tok)
        if m:
            d[m.group(1)] = int(m.group(2))
    return d


def parse_i2s_report(s):
    """I2S self-test report -> dict, None if s is not one."""
    t = s.split()
    if len(t) < 3 or not re.fullmatch(r"#\d+", t[0]) or t[2] not in I2S_FORMATS:
        return None
    d = {"raw": s, "seq": int(t[0][1:]), "state": t[1], "fmt": t[2], "pattern": True}
    rest = t[3:]
    m = _FILTER_TOKEN.match(rest[0]) if rest else None
    if m:
        d["flt"] = rest[0]
        d["filter"] = m.group(1) or m.group(3)
        d["osr"] = int(m.group(2)) if m.group(2) else None
        rest = rest[1:]
    for tok in rest:
        m = re.fullmatch(r"tx(\d+)(?:/(\d+))?", tok)
        if m:
            d["tx"] = (int(m.group(1)), int(m.group(2) or 0))
            continue
        m = re.fullmatch(r"lvl([0-9A-F])", tok)
        if m:
            d["lvl"] = int(m.group(1), 16)
            continue
        m = re.fullmatch(r"@(\d+):([0-9A-F]{8})>([0-9A-F]{8})/([0-9A-F]{8})", tok)
        if m:
            d["viol"] = (int(m.group(1)), m.group(2), m.group(3), m.group(4))
            continue
        if tok == "s-":
            d["pattern"] = False
            continue
        m = _INT_TOKEN.match(tok)
        if m:
            d[m.group(1)] = int(m.group(2))
    return d


def parse_diag(s):
    """Diagnostic string -> dict (boots, prev, app fields), None if not one."""
    head, sep, app = s.partition(" | ")
    m = re.match(r"^b(\d+) (\S+)", head)
    if not m or not sep:
        return None
    d = {"raw": s, "boots": int(m.group(1)), "prev": m.group(2), "app": app, "fields": {},
         "uac1": False}
    t = app.split()
    if t and re.fullmatch(r"[DIS][ao1]", t[0]):
        d["link"], d["host"] = t[0][0], t[0][1]
        d["uac1"] = d["host"] == "1"
        t = t[1:]
    for tok in t:
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


class Reader:
    def __init__(self, verbose=False):
        self.backend = libusb_package.get_libusb1_backend()
        self.verbose = verbose

    def read(self, tag=""):
        """(diag dict, S/PDIF report dict, I2S report dict, S/PDIF report raw);
        dicts are None if unreadable (the I2S dict carries its own raw)."""
        dg_raw = read_diag(self.backend, STRID_DIAG)
        st_raw = read_diag(self.backend, STRID_SELFTEST)
        i2_raw = read_diag(self.backend, STRID_SELFTEST_I2S)
        if self.verbose:
            print(f"    {tag:>8} {st_raw}  || {i2_raw}  || {dg_raw.partition(' | ')[2]}")
        return parse_diag(dg_raw), parse_report(st_raw), parse_i2s_report(i2_raw), st_raw


def ppm(fs, rate):
    return (fs - rate) * 1e6 / rate


def check_output(st, rate, bits, pattern=True):
    """Failure reasons for one report while the given format plays."""
    if st is None:
        return ["no self-test report"]
    f = []
    if st["state"] != "ok":
        f.append(f"state {st['state']}")
    if st.get("tx") != (rate, bits):
        f.append(f"checked against tx{st.get('tx')}")
    # Without the pattern, zero frames are ordinary audio.
    for k in (("z", "s") if pattern else ()) + ("e", "p", "o", "v"):
        if st.get(k, 0) != 0:
            f.append(f"{k}{st[k]}")
    if st.get("csflag") != "1" or st.get("cs3") != RATE_CODE[rate] or st.get("cs4") != LEN_CODE[bits]:
        f.append(f"channel status c{st.get('csflag')} "
                 f"{st.get('cs3', 0):02X}{st.get('cs4', 0):02X}")
    if abs(ppm(st.get("fs", 0), rate)) > MAX_PPM:
        f.append(f"fs{st.get('fs')} ({ppm(st.get('fs', 0), rate):+.0f} ppm)")
    if "viol" in st:
        f.append("first violation @%d: %s>%s/%s" % st["viol"])
    return f


def expected_osr(filt, rate):
    """Oversampling of a running dual-line output with filter filt."""
    if filt in I2S_OVERSAMPLING_FILTERS:
        return 8 if rate < 50000 else 4
    return None


def check_i2s(i2, rate, bits, fmt=None, pattern=True, filt=None):
    """Failure reasons for one I2S report while the given format plays; fmt /
    filt: the expected jumper format and filter names (None: any)."""
    if i2 is None:
        return ["no I2S self-test report"]
    f = []
    if i2["state"] != "ok":
        f.append(f"i2s state {i2['state']}")
    if fmt and i2["fmt"] != fmt:
        f.append(f"i2s format {i2['fmt']} (expected {fmt})")
    if filt and i2.get("filter") != filt:
        f.append(f"i2s filter {i2.get('flt')} (expected {filt})")
    if i2["state"] in ("ok", "sil", "ERR") and "flt" in i2:
        osr = expected_osr(i2["filter"], rate)
        if i2["osr"] != osr:
            f.append(f"i2s filter {i2['flt']} (expected {i2['filter']}{osr} at {rate} Hz)")
    if i2.get("tx") != (rate, bits):
        f.append(f"i2s checked against tx{i2.get('tx')}")
    # Zero frames are only an error against the pattern; a numeric s is
    # checked whenever the device ran a sample check (the oversampled
    # formats compare against the filtered S/PDIF samples in every build).
    for k in (("z",) if pattern else ()) + (("s",) if i2["pattern"] else ()) + ("e",):
        if i2.get(k, 0) != 0:
            f.append(f"i2s {k}{i2[k]}")
    if "viol" in i2:
        f.append("i2s first violation @%d: %s>%s/%s" % i2["viol"])
    return f


def check_device(dg, rate, bits, boots):
    """Failure reasons from the diagnostic string while playing."""
    if dg is None:
        return ["no diagnostic string"]
    f = []
    if dg["boots"] != boots:
        f.append(f"device rebooted (b{boots} -> b{dg['boots']}, previous boot: {dg['prev']})")
    if dg.get("link") != "S" or (dg.get("rate"), dg.get("bits")) != (rate, bits):
        f.append(f"stream {dg.get('link')} {dg.get('rate')}/{dg.get('bits')}")
    tx = dg["fields"].get("tx")
    if tx is not None and tx != rate:
        f.append(f"app tx{tx}")
    f += check_volume(dg)
    return f


def check_volume(dg):
    """Failure reasons from the device volume: bit-exact needs 0 dB, unmuted
    (firmware without a volume control reports none)."""
    vol = dg.get("volume") if dg else None
    if vol == "M":
        return ["device muted (vM): unmute it in Windows"]
    if vol:
        return [f"device volume -{vol} dB (v{vol}): set the Windows volume to 100 %"]
    return []


def counter_increases(a, b):
    """Increases of the device and window counters from read a to read b
    (each a (diag, S/PDIF report, I2S report) triple)."""
    f = []
    (dga, sta, i2a), (dgb, stb, i2b) = a, b
    if dga and dgb:
        for k in DEVICE_COUNTERS:
            if k in dga["fields"] and dgb["fields"].get(k, 0) > dga["fields"][k]:
                f.append(f"{k} {dga['fields'][k]} -> {dgb['fields'][k]}")
    if sta and stb:
        for k in WINDOW_COUNTERS:
            if stb.get(k, 0) > sta.get(k, 0):
                f.append(f"{k} {sta.get(k, 0)} -> {stb[k]}")
    if i2a and i2b:
        for k in I2S_COUNTERS:
            if i2b.get(k, 0) > i2a.get(k, 0):
                f.append(f"i2s {k} {i2a.get(k, 0)} -> {i2b[k]}")
    return f


def app_summary(dg):
    if not dg:
        return ""
    s = " ".join(f"{k}{v}" for k, v in dg["fields"].items() if k not in ("tx", "ff"))
    if "volume" in dg:
        s += f" v{dg['volume']}"
    if "trim" in dg:
        s += f" trim {dg['trim']:+d} ppm {'locked' if dg['locked'] else 'not locked'}"
    return s


def check_trim(dg):
    """UAC1: failure reasons from the adaptive clock trim."""
    if not dg or not dg["uac1"]:
        return []
    if "trim" not in dg:
        return ["no trim in the diagnostic string"]
    if abs(dg["trim"]) >= TRIM_LIMIT_PPM:
        return [f"trim {dg['trim']:+d} ppm at the servo limit"]
    return []


def wait_until(t0, at):
    time.sleep(max(0.0, at - (time.time() - t0)))


#--------------------------------------------------------------------------
# Sweep
#--------------------------------------------------------------------------

def play_item(rd, exe, device, rate, bits, boots, i2s_fmt, i2s_filter=None):
    """(S/PDIF failures, I2S failures, last read)."""
    player = make_player(exe, rate, bits, PLAY_SECONDS, device)
    t0 = time.time()
    reads = []
    for at in SAMPLE_AT:
        wait_until(t0, at)
        dg, st, i2, _ = rd.read(f"{at:.1f}s")
        reads.append((dg, st, i2))
    code, err = player.wait()

    fails, i2s_fails = [], []
    if code != 0:
        fails.append(f"player exit {code}: {err}")
    for (dg, st, i2), at in zip(reads, SAMPLE_AT):
        for reason in (check_device(dg, rate, bits, boots) + check_output(st, rate, bits) +
                       check_trim(dg)):
            fails.append(f"@{at:.1f}s {reason}")
        for reason in check_i2s(i2, rate, bits, i2s_fmt, filt=i2s_filter):
            i2s_fails.append(f"@{at:.1f}s {reason}")
    seqs = [st["seq"] for _, st, _ in reads if st]
    if any(a == b for a, b in zip(seqs, seqs[1:])):
        fails.append(f"stale reports (seq {seqs})")
    for reason in counter_increases(reads[0], reads[-1]):
        (i2s_fails if reason.startswith("i2s ") else fails).append(f"during playback: {reason}")
    return fails, i2s_fails, reads[-1]


def stop_test(rd, rate, boots):
    """Right after the last stream closed: silence, then the outputs stop."""
    t0 = time.time()
    fails = []
    wait_until(t0, STOP_SILENCE_AT)
    dg1, st1, i21, raw1 = rd.read(f"+{STOP_SILENCE_AT:.1f}s")
    if st1 is None:
        return ["no self-test report"], None, raw1
    if st1["state"] != "sil" or st1.get("z") != st1.get("f") or st1.get("tx", (0,))[0] != rate:
        fails.append(f"+{STOP_SILENCE_AT}s: expected silence at {rate} Hz: {raw1}")
    if i21 is None:
        fails.append(f"+{STOP_SILENCE_AT}s: no I2S self-test report")
    elif i21["state"] != "sil" or i21.get("z") != i21.get("f") or i21.get("tx", (0,))[0] != rate:
        fails.append(f"+{STOP_SILENCE_AT}s: expected I2S silence at {rate} Hz: {i21['raw']}")
    if dg1 and dg1["fields"].get("led", 0) != 0:
        fails.append(f"+{STOP_SILENCE_AT}s: LED still on")
    stopped_at = None
    while time.time() - t0 < STOP_DONE_AT - 0.2:
        time.sleep(0.2)
        _, st, _, _ = rd.read(f"+{time.time() - t0:.1f}s")
        if st and stopped_at is None and st.get("tx") == (0, 0):
            stopped_at = time.time() - t0
    wait_until(t0, STOP_DONE_AT)
    dg2, st2, i22, raw2 = rd.read(f"+{STOP_DONE_AT:.1f}s")
    if st2 is None:
        return fails + ["no self-test report"], stopped_at, raw2
    if st2["state"] != "idle" or st2.get("lvl") != 0 or st2.get("tx") != (0, 0):
        fails.append(f"+{STOP_DONE_AT}s: expected output stopped, line low: {raw2}")
    if i22 is None:
        fails.append(f"+{STOP_DONE_AT}s: no I2S self-test report")
    elif i22["state"] != "idle" or i22.get("lvl") != 0 or i22.get("tx") != (0, 0):
        fails.append(f"+{STOP_DONE_AT}s: expected I2S output stopped, pins low: {i22['raw']}")
    if st2["seq"] == st1["seq"]:
        fails.append("stale reports after the stop")
    if dg2 and dg2["boots"] != boots:
        fails.append("device rebooted")
    for k in ("B", "G", "L"):
        if k != "L" and st2.get(k, 0) != st1.get(k, 0):
            fails.append(f"{k} {st1.get(k, 0)} -> {st2.get(k, 0)} during silence / stop")
        if i21 and i22 and i22.get(k, 0) != i21.get(k, 0):
            fails.append(f"i2s {k} {i21.get(k, 0)} -> {i22.get(k, 0)} during silence / stop")
    return fails, stopped_at, raw2


def run_sweep(rd, exe, args, boots, uac1, i2s_fmt, i2s_filter=None):
    sweep = SWEEP_UAC1 if uac1 else SWEEP
    if args.only:
        rate, bits = (int(x) for x in args.only.split("/"))
        sweep = [(rate, bits)]
    results = []
    for rate, bits in sweep:
        fails, i2s_fails, (dg, st, i2) = play_item(rd, exe, args.device, rate, bits, boots, i2s_fmt,
                                                   i2s_filter)
        err = ppm(st["fs"], rate) if st and "fs" in st else float("nan")
        results.append((f"{rate}/{bits}", fails, i2s_fails, err, i2.get("flt", "") if i2 else ""))
        print(f"{rate:6d} Hz {bits}-bit: {'PASS' if not fails else 'FAIL'}  {err:+6.0f} ppm  "
              f"{st['raw'] if st else ''}")
        print(f"{'':16}i2s: {'PASS' if not i2s_fails else 'FAIL'}  {i2['raw'] if i2 else ''}")
        print(f"{'':16}app: {app_summary(dg)}")
        for reason in fails[:6] + i2s_fails[:6]:
            print(f"        {reason}")

    if not args.no_stop:
        rate = sweep[-1][0]
        fails, stopped_at, raw = stop_test(rd, rate, boots)
        when = f"stopped after {stopped_at:.1f} s" if stopped_at is not None else "no stop seen"
        results.append(("3-s stop", fails, [], float("nan"), ""))
        print(f"3-s stop: {'PASS' if not fails else 'FAIL'}  ({when})  {raw}")
        for reason in fails:
            print(f"        {reason}")

    print()
    print(f"{'item':<10} {'spdif':<6} {'ppm':>6}  {'i2s':<6} {'filter':<9} first failure")
    for name, fails, i2s_fails, err, flt in results:
        err_s = "" if err != err else f"{err:+.0f}"
        i2s_s = "" if name == "3-s stop" else "PASS" if not i2s_fails else "FAIL"
        first = fails[0] if fails else i2s_fails[0] if i2s_fails else ""
        print(f"{name:<10} {'PASS' if not fails else 'FAIL':<6} {err_s:>6}  {i2s_s:<6} {flt:<9} {first}")
    ok = all(not f and not i for _, f, i, _, _ in results)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


#--------------------------------------------------------------------------
# Soak
#--------------------------------------------------------------------------

def run_soak(rd, exe, args, boots, i2s_fmt, i2s_filter=None):
    rate, bits = args.rate, args.bits
    seconds = args.soak * 60.0
    flt = f" {i2s_filter}" if i2s_filter and i2s_fmt in DUAL_FORMATS else ""
    print(f"soak: {rate} Hz {bits}-bit, I2S {i2s_fmt}{flt}, for {args.soak:g} min, reading every "
          f"{args.interval:g} s")
    player = make_player(exe, rate, bits, SOAK_WARMUP + seconds + SOAK_MARGIN, args.device)
    t0 = time.time()
    events = []

    def log(msg):
        stamp = time.strftime("%H:%M:%S")
        events.append(msg)
        print(f"{stamp} +{time.time() - t0:7.1f}s  {msg}")

    trims = []
    unlocked = 0

    def note_clock(dg):
        nonlocal unlocked
        if dg and "trim" in dg:
            trims.append(dg["trim"])
            unlocked += not dg["locked"]

    try:
        wait_until(t0, SOAK_WARMUP)
        prev = rd.read("base")[:3]
        for reason in (check_device(prev[0], rate, bits, boots) +
                       check_output(prev[1], rate, bits) + check_trim(prev[0]) +
                       check_i2s(prev[2], rate, bits, i2s_fmt, filt=i2s_filter)):
            log(reason)
        next_at, next_progress = SOAK_WARMUP + args.interval, 60.0
        while next_at <= SOAK_WARMUP + seconds:
            wait_until(t0, next_at)
            next_at += args.interval
            if player.proc.poll() is not None:
                log("player ended early")
                break
            cur = rd.read(f"{time.time() - t0:.0f}s")[:3]
            dg, st, i2 = cur
            note_clock(dg)
            for reason in (check_device(dg, rate, bits, boots) + check_output(st, rate, bits) +
                           check_trim(dg) + check_i2s(i2, rate, bits, i2s_fmt, filt=i2s_filter)):
                log(reason)
            for reason in counter_increases(prev, cur):
                log(reason)
            if st and prev[1] and st["seq"] == prev[1]["seq"]:
                log(f"stale report (seq {st['seq']})")
            if st and dg and i2:
                prev = cur
            if time.time() - t0 >= next_progress:
                next_progress += 60.0
                print(f"{time.strftime('%H:%M:%S')} +{time.time() - t0:7.1f}s  "
                      f"{len(events)} events  {st['raw'] if st else ''}  "
                      f"{i2['raw'] if i2 else ''}  | {app_summary(dg)}")
        code, err = player.wait()
        if code != 0:
            log(f"player exit {code}: {err}")
    except KeyboardInterrupt:
        player.kill()
        log("interrupted")

    elapsed = (time.time() - t0) / 60.0
    ok = not events
    if trims:
        print(f"adaptive clock: trim {min(trims):+d} .. {max(trims):+d} ppm, "
              f"{unlocked} of {len(trims)} reads not locked")
    print(f"soak {rate}/{bits}: {'PASS' if ok else 'FAIL'}, {elapsed:.1f} min, "
          f"{len(events)} events")
    return ok


#--------------------------------------------------------------------------
# Test tone build
#--------------------------------------------------------------------------

def run_tone(rd, boots, i2s_fmt, i2s_filter=None):
    """Tone build: framing, channel status and rate of the 48 kHz tone, and
    the I2S framing."""
    print(f"test-tone build: checking framing, channel status and rate at "
          f"{TONE_RATE} Hz {TONE_BITS}-bit (no pattern)")
    reads = []
    for i in range(5):
        if i:
            time.sleep(1.0)
        dg, st, i2, raw = rd.read(f"{i}")
        reads.append((dg, st, i2))
        print(f"  {raw}  {i2['raw'] if i2 else ''}")
    fails = []
    for dg, st, i2 in reads:
        fails += check_output(st, TONE_RATE, TONE_BITS, pattern=False)
        fails += check_i2s(i2, TONE_RATE, TONE_BITS, i2s_fmt, pattern=False, filt=i2s_filter)
        if dg and dg["boots"] != boots:
            fails.append("device rebooted")
    seqs = [st["seq"] for _, st, _ in reads if st]
    if any(a == b for a, b in zip(seqs, seqs[1:])):
        fails.append(f"stale reports (seq {seqs})")
    fails += counter_increases(reads[0], reads[-1])
    for reason in fails:
        print(f"    {reason}")
    print("PASS" if not fails else "FAIL")
    return not fails


#--------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-v", "--verbose", action="store_true", help="print every read")
    ap.add_argument("--only", metavar="RATE/BITS", help="sweep one format, e.g. 44100/16")
    ap.add_argument("--no-stop", action="store_true", help="skip the 3-s stop test")
    ap.add_argument("--soak", type=float, metavar="MINUTES", help="soak one format")
    ap.add_argument("--rate", type=int, default=48000, choices=sorted(RATE_CODE),
                    help="soak sample rate (default 48000)")
    ap.add_argument("--bits", type=int, choices=(16, 24),
                    help="soak bit depth (default 24; 16 in UAC1 mode)")
    ap.add_argument("--interval", type=float, default=5.0, metavar="S",
                    help="soak read interval (default 5 s)")
    ap.add_argument("--device", help="playback device name part (default: the bridge in either mode)")
    ap.add_argument("--i2s-format", choices=I2S_FORMATS, metavar="FMT",
                    help="expected I2S format, one of " + ", ".join(I2S_FORMATS) +
                         " (default: what the device reports)")
    ap.add_argument("--i2s-filter", choices=I2S_FILTERS, metavar="NAME",
                    help="expected dual-line filter, one of " + ", ".join(I2S_FILTERS) +
                         " (default: what the device reports)")
    ap.add_argument("--status", action="store_true", help="print the three reports once and exit")
    args = ap.parse_args()

    rd = Reader(args.verbose)
    dg, st, i2, raw = rd.read("start")
    for _ in range(10):     # first window after boot
        if not st or st["state"] != "wait":
            break
        time.sleep(0.2)
        dg, st, i2, raw = rd.read("start")
    if args.status:
        print(dg["raw"] if dg else "<no diagnostic string>")
        print(raw)
        print(i2["raw"] if i2 else "<no I2S self-test report>")
        return
    if dg is None:
        sys.exit(f"usb2spdif not found or not responding: {raw}")
    if st is None:
        sys.exit(f"no self-test report ({raw}): flash a build with -DUSB2SPDIF_SELFTEST=ON")
    if i2 is None:
        sys.exit("no I2S self-test report: flash a build with -DUSB2SPDIF_SELFTEST=ON")
    boots = dg["boots"]
    uac1 = dg["uac1"]
    i2s_fmt = args.i2s_format or i2["fmt"]
    i2s_filter = args.i2s_filter or i2.get("filter")
    if i2s_filter == "-":
        i2s_filter = None       # direct format: nothing to require
    print(f"device: {dg['raw']}")
    print(f"report: {raw}")
    print(f"i2s:    {i2['raw']}")
    print(f"mode:   {'UAC1 (adaptive, 16 bit, up to 48 kHz)' if uac1 else 'UAC2'}, "
          f"I2S format {i2s_fmt}, filter {i2.get('flt', '?')}")
    if args.bits is None:
        args.bits = 16 if uac1 else 24
    formats = [(args.rate, args.bits)] if args.soak else []
    if args.only:
        formats.append(tuple(int(x) for x in args.only.split("/")))
    if uac1 and st["pattern"] and any(r not in UAC1_RATES or b != 16 for r, b in formats):
        sys.exit(f"UAC1 mode offers 16 bit at {', '.join(map(str, UAC1_RATES))} Hz only")
    if st["state"] == "NOPIO" or i2["state"] == "NOPIO":
        sys.exit("self-test could not claim a PIO state machine or DMA channel")
    if not st["pattern"]:
        sys.exit(0 if run_tone(rd, boots, i2s_fmt, i2s_filter) else 1)

    if sys.platform == "win32":
        try:
            exe = build_player()
        except PlayerBuildError as e:
            sys.exit(str(e))
        set_volume_max(exe, args.device)
    else:
        exe = alsa_card(args.device)
        if exe is None:
            sys.exit("no ALSA card with the bridge's name (/proc/asound/cards)")
        print(f"alsa:   card {exe} (hw:{exe},0)")
        alsa_volume_max(exe)
    if args.soak:
        ok = run_soak(rd, exe, args, boots, i2s_fmt, i2s_filter)
    else:
        ok = run_sweep(rd, exe, args, boots, uac1, i2s_fmt, i2s_filter)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
