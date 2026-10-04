"""usb2spdif self-test: bit-exact check of the S/PDIF output (Windows).

Needs firmware built with -DUSB2SPDIF_SELFTEST=ON. The Pico then decodes
its own S/PDIF pin on core 1 and reports the result as USB string 0x21; no
receiver, probe or UART is needed. This script plays a test pattern with a
native WASAPI exclusive-mode player (tools/wasapi_play.c, built on first use
with the ziglang package into tools/build/) and reads the reports.

    pip install -r tools/requirements.txt
    python tools/selftest.py                     sweep: 5 rates x 16/24 bit, 3-s stop
    python tools/selftest.py --only 44100/16     one format
    python tools/selftest.py --soak 60 --rate 96000 --bits 24
                                                 one format for 60 minutes
    python tools/selftest.py --status            print both reports once

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
status and rate of the tone only.

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

The diagnostic string (tools/diag.py) supplies the boot count and the
application status; its name<number> tokens are parsed generically.
"""
import argparse
import os
import re
import subprocess
import sys
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
        """(diag dict, report dict, report raw); dicts are None if unreadable."""
        dg_raw = read_diag(self.backend, STRID_DIAG)
        st_raw = read_diag(self.backend, STRID_SELFTEST)
        if self.verbose:
            print(f"    {tag:>8} {st_raw}  || {dg_raw.partition(' | ')[2]}")
        return parse_diag(dg_raw), parse_report(st_raw), st_raw


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
    """Increases of the device and window counters from read a to read b."""
    f = []
    (dga, sta), (dgb, stb) = a, b
    if dga and dgb:
        for k in DEVICE_COUNTERS:
            if k in dga["fields"] and dgb["fields"].get(k, 0) > dga["fields"][k]:
                f.append(f"{k} {dga['fields'][k]} -> {dgb['fields'][k]}")
    if sta and stb:
        for k in WINDOW_COUNTERS:
            if stb.get(k, 0) > sta.get(k, 0):
                f.append(f"{k} {sta.get(k, 0)} -> {stb[k]}")
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

def play_item(rd, exe, device, rate, bits, boots):
    player = Player(exe, rate, bits, PLAY_SECONDS, device)
    t0 = time.time()
    reads = []
    for at in SAMPLE_AT:
        wait_until(t0, at)
        dg, st, _ = rd.read(f"{at:.1f}s")
        reads.append((dg, st))
    code, err = player.wait()

    fails = []
    if code != 0:
        fails.append(f"player exit {code}: {err}")
    for (dg, st), at in zip(reads, SAMPLE_AT):
        for reason in (check_device(dg, rate, bits, boots) + check_output(st, rate, bits) +
                       check_trim(dg)):
            fails.append(f"@{at:.1f}s {reason}")
    seqs = [st["seq"] for _, st in reads if st]
    if any(a == b for a, b in zip(seqs, seqs[1:])):
        fails.append(f"stale reports (seq {seqs})")
    for reason in counter_increases(reads[0], reads[-1]):
        fails.append(f"during playback: {reason}")
    return fails, reads[-1]


def stop_test(rd, rate, boots):
    """Right after the last stream closed: silence, then the output stops."""
    t0 = time.time()
    fails = []
    wait_until(t0, STOP_SILENCE_AT)
    dg1, st1, raw1 = rd.read(f"+{STOP_SILENCE_AT:.1f}s")
    if st1 is None:
        return ["no self-test report"], None, raw1
    if st1["state"] != "sil" or st1.get("z") != st1.get("f") or st1.get("tx", (0,))[0] != rate:
        fails.append(f"+{STOP_SILENCE_AT}s: expected silence at {rate} Hz: {raw1}")
    if dg1 and dg1["fields"].get("led", 0) != 0:
        fails.append(f"+{STOP_SILENCE_AT}s: LED still on")
    stopped_at = None
    while time.time() - t0 < STOP_DONE_AT - 0.2:
        time.sleep(0.2)
        _, st, _ = rd.read(f"+{time.time() - t0:.1f}s")
        if st and stopped_at is None and st.get("tx") == (0, 0):
            stopped_at = time.time() - t0
    wait_until(t0, STOP_DONE_AT)
    dg2, st2, raw2 = rd.read(f"+{STOP_DONE_AT:.1f}s")
    if st2 is None:
        return fails + ["no self-test report"], stopped_at, raw2
    if st2["state"] != "idle" or st2.get("lvl") != 0 or st2.get("tx") != (0, 0):
        fails.append(f"+{STOP_DONE_AT}s: expected output stopped, line low: {raw2}")
    if st2["seq"] == st1["seq"]:
        fails.append("stale reports after the stop")
    if dg2 and dg2["boots"] != boots:
        fails.append("device rebooted")
    for k in ("B", "G"):
        if st2.get(k, 0) != st1.get(k, 0):
            fails.append(f"{k} {st1.get(k, 0)} -> {st2.get(k, 0)} during silence / stop")
    return fails, stopped_at, raw2


def run_sweep(rd, exe, args, boots, uac1):
    sweep = SWEEP_UAC1 if uac1 else SWEEP
    if args.only:
        rate, bits = (int(x) for x in args.only.split("/"))
        sweep = [(rate, bits)]
    results = []
    for rate, bits in sweep:
        fails, (dg, st) = play_item(rd, exe, args.device, rate, bits, boots)
        err = ppm(st["fs"], rate) if st and "fs" in st else float("nan")
        results.append((f"{rate}/{bits}", fails, err))
        print(f"{rate:6d} Hz {bits}-bit: {'PASS' if not fails else 'FAIL'}  {err:+6.0f} ppm  "
              f"{st['raw'] if st else ''}")
        print(f"{'':20}app: {app_summary(dg)}")
        for reason in fails[:6]:
            print(f"        {reason}")

    if not args.no_stop:
        rate = sweep[-1][0]
        fails, stopped_at, raw = stop_test(rd, rate, boots)
        when = f"stopped after {stopped_at:.1f} s" if stopped_at is not None else "no stop seen"
        results.append(("3-s stop", fails, float("nan")))
        print(f"3-s stop: {'PASS' if not fails else 'FAIL'}  ({when})  {raw}")
        for reason in fails:
            print(f"        {reason}")

    print()
    print(f"{'item':<10} {'result':<6} {'ppm':>6}  first failure")
    for name, fails, err in results:
        err_s = "" if err != err else f"{err:+.0f}"
        print(f"{name:<10} {'PASS' if not fails else 'FAIL':<6} {err_s:>6}  "
              f"{fails[0] if fails else ''}")
    ok = all(not f for _, f, _ in results)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


#--------------------------------------------------------------------------
# Soak
#--------------------------------------------------------------------------

def run_soak(rd, exe, args, boots):
    rate, bits = args.rate, args.bits
    seconds = args.soak * 60.0
    print(f"soak: {rate} Hz {bits}-bit for {args.soak:g} min, reading every "
          f"{args.interval:g} s")
    player = Player(exe, rate, bits, SOAK_WARMUP + seconds + SOAK_MARGIN, args.device)
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
        prev = rd.read("base")[:2]
        for reason in (check_device(prev[0], rate, bits, boots) +
                       check_output(prev[1], rate, bits) + check_trim(prev[0])):
            log(reason)
        next_at, next_progress = SOAK_WARMUP + args.interval, 60.0
        while next_at <= SOAK_WARMUP + seconds:
            wait_until(t0, next_at)
            next_at += args.interval
            if player.proc.poll() is not None:
                log("player ended early")
                break
            cur = rd.read(f"{time.time() - t0:.0f}s")[:2]
            dg, st = cur
            note_clock(dg)
            for reason in (check_device(dg, rate, bits, boots) + check_output(st, rate, bits) +
                           check_trim(dg)):
                log(reason)
            for reason in counter_increases(prev, cur):
                log(reason)
            if st and prev[1] and st["seq"] == prev[1]["seq"]:
                log(f"stale report (seq {st['seq']})")
            if st and dg:
                prev = cur
            if time.time() - t0 >= next_progress:
                next_progress += 60.0
                print(f"{time.strftime('%H:%M:%S')} +{time.time() - t0:7.1f}s  "
                      f"{len(events)} events  {st['raw'] if st else ''}  | {app_summary(dg)}")
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

def run_tone(rd, boots):
    """Tone build: framing, channel status and rate of the 48 kHz tone."""
    print(f"test-tone build: checking framing, channel status and rate at "
          f"{TONE_RATE} Hz {TONE_BITS}-bit (no pattern)")
    reads = []
    for i in range(5):
        if i:
            time.sleep(1.0)
        dg, st, raw = rd.read(f"{i}")
        reads.append((dg, st))
        print(f"  {raw}")
    fails = []
    for dg, st in reads:
        fails += check_output(st, TONE_RATE, TONE_BITS, pattern=False)
        if dg and dg["boots"] != boots:
            fails.append("device rebooted")
    seqs = [st["seq"] for _, st in reads if st]
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
    ap.add_argument("--status", action="store_true", help="print both reports once and exit")
    args = ap.parse_args()

    rd = Reader(args.verbose)
    dg, st, raw = rd.read("start")
    for _ in range(10):     # first window after boot
        if not st or st["state"] != "wait":
            break
        time.sleep(0.2)
        dg, st, raw = rd.read("start")
    if args.status:
        print(dg["raw"] if dg else "<no diagnostic string>")
        print(raw)
        return
    if dg is None:
        sys.exit(f"usb2spdif not found or not responding: {raw}")
    if st is None:
        sys.exit(f"no self-test report ({raw}): flash a build with -DUSB2SPDIF_SELFTEST=ON")
    boots = dg["boots"]
    uac1 = dg["uac1"]
    print(f"device: {dg['raw']}")
    print(f"report: {raw}")
    print(f"mode:   {'UAC1 (adaptive, 16 bit, up to 48 kHz)' if uac1 else 'UAC2'}")
    if args.bits is None:
        args.bits = 16 if uac1 else 24
    formats = [(args.rate, args.bits)] if args.soak else []
    if args.only:
        formats.append(tuple(int(x) for x in args.only.split("/")))
    if uac1 and st["pattern"] and any(r not in UAC1_RATES or b != 16 for r, b in formats):
        sys.exit(f"UAC1 mode offers 16 bit at {', '.join(map(str, UAC1_RATES))} Hz only")
    if st["state"] == "NOPIO":
        sys.exit("self-test could not claim a PIO state machine or DMA channel")
    if not st["pattern"]:
        sys.exit(0 if run_tone(rd, boots) else 1)

    if sys.platform != "win32":
        sys.exit("the pattern player needs Windows (WASAPI)")
    try:
        exe = build_player()
    except PlayerBuildError as e:
        sys.exit(str(e))
    set_volume_max(exe, args.device)
    if args.soak:
        ok = run_soak(rd, exe, args, boots)
    else:
        ok = run_sweep(rd, exe, args, boots, uac1)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
