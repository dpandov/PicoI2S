"""Live monitor of the usb2spdif bridge: sessions, format, data rate, silence.

Polls the device's diagnostic string (tools/diag.py) and its session string
a few times per second and prints one timestamped line per event:

    device connected / disconnected / rebooted (with the crash record and
        the configuration: board, firmware, build, USB mode, output format,
        filter, jumper states)
    stream opened (format, PCM data rate, volume) / closed / format changed
    playing (stream samples going out) / stalled / underrun
    S/PDIF output on / off (3 s after the audio ended)
    digital silence (all-zero samples for --silence seconds) / signal
    volume changes, clock lock (UAC1 and the S/PDIF input), counter increases
    (ov, lt, late I2S blocks)
    S/PDIF input: lock / loss of lock per input (coax, Toslink), the input
        selected and its rate, playing / stopped playing, the output
        changing hands between USB and the input, pre-emphasis on / off,
        non-PCM data, receiver error counter increases

With --status N a status line is printed every N seconds while a stream is
open or the input plays: format, data rate, the sample rate measured from
the device's frame counter against the PC clock, FIFO level, ring minimum,
longest main-loop iteration, trim and the current silence (for the input:
its rate and offset, word length, emphasis and the silence).

    pip install pyusb libusb-package
    python tools/monitor.py                      # until Ctrl+C
    python tools/monitor.py --status 10 --log session.log
    python tools/monitor.py --silence 2 --interval 0.5

Needs the firmware's session string (index 0x23); the configuration
string (0x24) is optional (older firmware: "not reported"). Short events between two
polls (a stream opened and closed within --interval) are not seen.
"""
import argparse
import sys
import time

import libusb_package

from diag import (STRID_CONFIG, STRID_DIAG, STRID_SESSION, parse_config, parse_session,
                  parse_status, read_diag)

# Measured sample rate: frame counter against the PC clock over this window,
# reported once half of it is filled. The USB read latency limits it to
# roughly 100 ppm at 30 s.
MEASURE_WINDOW_S = 30.0
MEASURE_WARMUP_S = 3.0    # the FIFO settles to its set point after the start


def fmt_duration(seconds):
    if seconds < 60:
        return f"{seconds:.1f} s"
    m, s = divmod(int(round(seconds)), 60)
    if m < 60:
        return f"{m} min {s:02d} s"
    h, m = divmod(m, 60)
    return f"{h} h {m:02d} min"


def fmt_format(rate, bits):
    if not rate:
        return "no format"
    mbit = rate * bits * 2 / 1e6
    return f"{rate} Hz / {bits}-bit ({mbit:.3f} Mbit/s)"


INPUTS = {"coax": "coax", "toslink": "Toslink"}


def fmt_input(se):
    """Selected input's rate, offset and channel status, from the session."""
    if not se.get("input_rate"):
        return "no rate"
    s = f"{se['input_rate']} Hz, {se['input_ppm']:+d} ppm"
    cs = se.get("cs") or {}
    if cs:
        s += f", {cs['bits']}-bit"
        if cs["emphasis"]:
            s += ", pre-emphasis"
        if not cs["pcm"]:
            s += ", non-PCM"
    return s


def fmt_volume(v):
    if v is None:
        return "?"
    return "muted" if v == "M" else ("0 dB" if v == 0 else f"-{v} dB")


BOARDS = {"rp2350-one": "Waveshare RP2350-One", "pico2": "Raspberry Pi Pico 2"}
BUILDS = {"normal": "normal build", "selftest": "self-test build", "tone": "test-tone build",
          "tone+selftest": "test-tone + self-test build"}
FORMATS = {
    "i2s": "I2S (Philips)", "lj": "left-justified", "rj24": "right-justified 24-bit",
    "rj16": "right-justified 16-bit",
    "dual16": "dual-line 16-bit (TDA1541A simultaneous mode), DEM clock on",
    "dual18": "dual-line 18-bit (AD1865)", "dual20": "dual-line 20-bit (PCM63 / PCM1702)",
    "dual24": "dual-line 24-bit",
}
FILTERS = {
    "nos": "NOS, no digital filter",
    "sharp": "sharp, linear phase, 8x at 32 / 44.1 / 48 kHz, 4x at 88.2 / 96 kHz",
    "slow": "slow roll-off, linear phase, 8x at 32 / 44.1 / 48 kHz, 4x at 88.2 / 96 kHz",
    "sinc": "NOS with sin(x)/x compensation, at the sample rate",
    "minphase": "minimum phase, 8x at 32 / 44.1 / 48 kHz, 4x at 88.2 / 96 kHz",
}
FILTER_CODES = ["nos", "sharp", "slow", "sinc", "minphase"]


def config_lines(cfg):
    """The configuration string's fields as report lines."""
    jumpers = cfg.get("jumpers", "?/???/???").split("/")
    mode_j, fmt_j, flt_j = (jumpers + ["?", "???", "???"])[:3]
    fw = cfg.get("fw", "")
    fw = f"{int(fw[:2], 16)}.{fw[2:]}" if len(fw) == 4 and all(c in "0123456789abcdef" for c in fw) else fw
    fmt, flt = cfg.get("format", "?"), cfg.get("filter", "?")
    lines = [
        f"board {BOARDS.get(cfg.get('board'), cfg.get('board'))}, firmware {fw}, "
        f"{BUILDS.get(cfg.get('build'), cfg.get('build'))}",
        f"USB {cfg.get('mode', '?').upper()} (mode jumper GP5 {'grounded' if mode_j == '1' else 'open'})",
        f"output format {FORMATS.get(fmt, fmt)} (jumpers GP8 GP7 GP6 = {fmt_j})",
    ]
    code = int(flt_j, 2) if set(flt_j) <= {"0", "1"} and flt_j else None
    reserved = code is not None and code >= len(FILTER_CODES)
    if fmt.startswith("dual"):
        lines.append(f"filter {FILTERS.get(flt, flt)} (jumpers GP2 GP3 GP4 = {flt_j}"
                     f"{', a reserved code' if reserved else ''})")
    else:
        lines.append(f"filter none, I2S / LJ / RJ are never filtered (jumpers GP2 GP3 GP4 = {flt_j}: "
                     f"{'reserved, sharp' if reserved else flt} for the dual-line formats)")
    return lines


class Monitor:
    def __init__(self, args):
        self.args = args
        self.backend = libusb_package.get_libusb1_backend()
        self.log = open(args.log, "a", encoding="utf-8") if args.log else None
        self.prev = None          # last parsed status
        self.sess = None          # last parsed session
        self.present = None
        self.open_since = None
        self.play_since = None
        self.silent_since = None
        self.tx_since = None
        self.input_since = None   # the S/PDIF input playing since
        self.prev_owner = None    # last source heard
        self.streams_total = 0
        self.streaming_total = 0.0
        self.frames_hist = []     # (t, frames) while playing, for the measured rate
        self.last_status_t = 0.0

    def out(self, msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        if self.log:
            self.log.write(line + "\n")
            self.log.flush()

    def read(self):
        raw = read_diag(self.backend, STRID_DIAG)
        st = parse_status(raw)
        if st is None:
            return None, None, raw
        se = parse_session(read_diag(self.backend, STRID_SESSION))
        return st, se, raw

    # ---- events ---------------------------------------------------------

    def device_change(self, st, raw):
        present = st is not None
        if present == self.present:
            return
        if present:
            mode = "UAC1" if st["uac1"] else "UAC2"
            host = {"a": "Apple host", "o": "non-Apple host", "1": "adaptive"}[st["host"]]
            self.out(f"device connected: {mode}, {host}, boot {st['boots']}")
            if st["prev"] != "none":
                self.out(f"  previous boot ended in: {st['prev']} ({raw.split(' | ')[0]})")
            self.config_report()
        elif self.present is not None:
            self.out(f"device disconnected ({raw})")
            self.close_stream_state()
        self.present = present

    def close_stream_state(self):
        now = time.monotonic()
        if self.open_since is not None:
            self.streaming_total += now - self.open_since
        self.open_since = None
        self.play_since = None
        self.silent_since = None
        self.frames_hist.clear()

    def stream_events(self, st, se, now):
        p = self.prev
        f = st["fields"]
        was_open = p is not None and p["link"] == "S"
        is_open = st["link"] == "S"
        fmt_changed = p is not None and (p["rate"], p["bits"]) != (st["rate"], st["bits"])
        starts = 0
        if se and self.sess:
            starts = (se["streams"] - self.sess["streams"]) & 0xFFFFFFFF

        if is_open and not was_open:
            self.out(f"stream opened: {fmt_format(st['rate'], st['bits'])}, "
                     f"volume {fmt_volume(st['volume'])}")
            self.open_since = now
            self.streams_total += 1
            if not (se and se.get("source") in INPUTS):
                self.silent_since = None
        elif is_open and (fmt_changed or starts > 0):
            self.out(f"stream restarted: {fmt_format(st['rate'], st['bits'])}, "
                     f"volume {fmt_volume(st['volume'])}"
                     + (f" ({starts} starts)" if starts > 1 else ""))
            self.streams_total += 1
            self.frames_hist.clear()
        elif was_open and not is_open:
            dur = fmt_duration(now - self.open_since) if self.open_since else "?"
            self.out(f"stream closed after {dur}")
            self.close_stream_state()
        elif fmt_changed and p is not None:
            self.out(f"format set to {fmt_format(st['rate'], st['bits'])} (stream closed)")

        if p is not None and st["link"] == "D" and p["link"] != "D":
            self.out("host detached (not configured)")

        run, prev_run = f.get("run", 0), (p["fields"].get("run", 0) if p else 0)
        if run and not prev_run:
            self.out("playing")
            self.play_since = now
        elif prev_run and not run:
            if is_open and se and se.get("source") in INPUTS:
                pass    # the input took the output: input_events() reports it
            elif is_open:
                ur_inc = f.get("ur", 0) - (p["fields"].get("ur", 0) if p else 0)
                if ur_inc > 0:
                    self.out(f"underrun: output ring ran dry ({f.get('ur', 0)} this stream)")
                else:
                    self.out("stalled: stream open, no samples arriving")
            self.play_since = None
            self.frames_hist.clear()

        tx, prev_tx = f.get("tx", 0), (p["fields"].get("tx", 0) if p else 0)
        if tx and not prev_tx:
            self.out(f"S/PDIF output on: {tx} Hz")
            self.tx_since = now
        elif prev_tx and not tx:
            dur = fmt_duration(now - self.tx_since) if self.tx_since else "?"
            self.out(f"S/PDIF output off after {dur} (3 s without audio)")
            self.tx_since = None
        elif tx and prev_tx and tx != prev_tx:
            self.out(f"S/PDIF output switched to {tx} Hz")

        for key, name in (("ov", "overrun"), ("lt", "late S/PDIF interrupt")):
            inc = f.get(key, 0) - (p["fields"].get(key, 0) if p else 0)
            if p is not None and is_open and inc > 0:
                self.out(f"{name}: +{inc} ({f.get(key, 0)} this stream)")

        if (se and self.sess and se.get("i2s_late") is not None
                and self.sess.get("i2s_late") is not None):
            inc = (se["i2s_late"] - self.sess["i2s_late"]) & 0xFFFFFFFF
            if inc > 0:
                self.out(f"late I2S block: +{inc} ({se['i2s_late']} since boot)")

        if p is not None and st["volume"] != p["volume"]:
            self.out(f"volume {fmt_volume(st['volume'])}")

        # The input's trim counts as playing too (run is USB playback).
        trimmed_play = run or (se and se.get("source") in ("coax", "toslink"))
        if st["trim"] is not None and p is not None and st["locked"] != p["locked"] and trimmed_play:
            if st["locked"]:
                self.out(f"clock locked, trim {st['trim']:+d} ppm")
            else:
                self.out(f"clock unlocked, trim {st['trim']:+d} ppm")

    def input_events(self, se, now):
        """S/PDIF input: locks, selection, source changes, flags, errors."""
        if se is None or se["source"] is None:
            return    # firmware without the input tokens
        q = self.sess if self.sess is not None and self.sess["source"] is not None else None
        prev_locked = q["locked"] if q else {"coax": False, "toslink": False}
        prev_sel = q["input"] if q else "none"
        prev_rate = q["input_rate"] if q else 0
        sel = se["input"]
        announced = False
        for name, label in INPUTS.items():
            if se["locked"][name] == prev_locked[name]:
                continue
            if not se["locked"][name]:
                self.out(f"{label} input lost lock")
            elif sel == name and prev_sel != name:
                self.out(f"{label} input locked and selected: {fmt_input(se)}")
                announced = True
            else:
                self.out(f"{label} input locked")
        if not announced and (sel, se["input_rate"]) != (prev_sel, prev_rate):
            if sel == "none":
                if prev_sel != "none":
                    self.out("no S/PDIF input selected")
            else:
                self.out(f"{INPUTS[sel]} input selected: {fmt_input(se)}")

        cs, prev_cs = se["cs"] or {}, (q["cs"] or {}) if q else {}
        emph, prev_emph = bool(cs) and cs["emphasis"], bool(prev_cs) and prev_cs["emphasis"]
        if emph != prev_emph:
            self.out("input pre-emphasis on (50/15 us)" if emph else "input pre-emphasis off")
        data, prev_data = bool(cs) and not cs["pcm"], bool(prev_cs) and not prev_cs["pcm"]
        if data != prev_data:
            self.out("non-PCM data on the input (played as silence)" if data else "PCM data on the input")

        src, prev_src = se["source"], (q["source"] if q else "none")
        if src != prev_src:
            if src in INPUTS:
                took = " (USB idle for 3 s)" if prev_src == "usb" or self.prev_owner == "usb" else ""
                self.out(f"playing: {INPUTS[src]} input, {fmt_input(se)}{took}")
                self.input_since = now
                self.silent_since = None
            elif prev_src in INPUTS:
                dur = fmt_duration(now - self.input_since) if self.input_since else "?"
                if src == "usb":
                    self.out(f"USB took the output back from the {INPUTS[prev_src]} input after {dur}")
                else:
                    self.out(f"{INPUTS[prev_src]} input stopped playing after {dur} "
                             "(lost lock, discontinuity or no samples)")
                self.input_since = None
                self.silent_since = None
        if src != "none":
            self.prev_owner = src

        if q and se["rx"] and q["rx"]:
            inc = {k: (se["rx"][k] - q["rx"][k]) % 100000 for k in ("parity", "framing", "overflows")}
            if any(inc.values()):
                self.out("input errors: " + ", ".join(f"+{v} {k}" for k, v in inc.items() if v)
                         + f" (since boot: {se['rx']['parity']} parity, {se['rx']['framing']} framing, "
                         f"{se['rx']['overflows']} overflows)")

    def silence_events(self, st, se, now):
        if se is None:
            return
        on_input = se.get("source") in INPUTS
        if not on_input and (st["link"] != "S" or not st["fields"].get("run", 0)):
            return
        start = self.input_since if on_input else self.open_since
        age = se["signal_age_ms"]
        silent = age is None or age >= self.args.silence * 1000.0
        if silent and self.silent_since is None:
            # The silence began when the last non-zero sample went out, or
            # with this stream (or input) if that was before it started.
            began = now - (age / 1000.0 if age is not None else 0.0)
            from_open = start is not None and began <= start + self.args.interval
            if from_open:
                began = start
            self.silent_since = began
            what = " since the input started playing" if on_input else " since the stream opened"
            self.out("digital silence" + (what if from_open else ""))
        elif not silent and self.silent_since is not None:
            self.out(f"signal after {fmt_duration(now - self.silent_since)} of silence")
            self.silent_since = None

    def reboot_events(self, st, raw):
        if self.prev is not None and st["boots"] != self.prev["boots"]:
            self.out(f"device rebooted: boot {st['boots']}, previous ended in {st['prev']}"
                     + ("" if st["prev"] == "none" else f" ({raw.split(' | ')[0]})"))
            self.close_stream_state()
            # The session counters restarted: compare against nothing.
            self.sess = None
            self.input_since = None
            self.config_report()

    def config_report(self):
        cfg = parse_config(read_diag(self.backend, STRID_CONFIG))
        if cfg is None:
            self.out("  config: not reported (firmware without the configuration string)")
            return
        for line in config_lines(cfg):
            self.out(f"  config: {line}")

    # ---- measured rate and status ---------------------------------------

    def track_frames(self, st, se, now):
        if (se is None or not st["fields"].get("run", 0) or self.play_since is None
                or now - self.play_since < MEASURE_WARMUP_S):
            return
        # Frames received from the host: moved into the ring plus those still
        # in the FIFO, so the regulated FIFO level's wander drops out.
        frame_bytes = 2 * (4 if st["bits"] == 24 else 2)
        self.frames_hist.append((now, se["frames"] + st["fields"].get("ff", 0) / frame_bytes))
        while self.frames_hist and now - self.frames_hist[0][0] > MEASURE_WINDOW_S:
            self.frames_hist.pop(0)

    def measured_rate(self):
        if len(self.frames_hist) < 2:
            return None
        (t0, f0), (t1, f1) = self.frames_hist[0], self.frames_hist[-1]
        if t1 - t0 < MEASURE_WINDOW_S / 2:
            return None
        return ((f1 - f0) % 4294967296.0) / (t1 - t0)

    def status_line(self, st, se, now):
        on_input = se is not None and se.get("source") in INPUTS
        if not self.args.status or (st["link"] != "S" and not on_input):
            return
        if now - self.last_status_t < self.args.status:
            return
        self.last_status_t = now
        if on_input:
            parts = [f"{INPUTS[se['source']]} input", fmt_input(se)]
            if self.silent_since is not None:
                parts.append(f"silence {fmt_duration(now - self.silent_since)}")
            self.out("status: " + ", ".join(parts))
            return
        f = st["fields"]
        rate = self.measured_rate()
        parts = [fmt_format(st["rate"], st["bits"])]
        if rate is not None:
            parts.append(f"measured {rate:.1f} Hz ({(rate / st['rate'] - 1) * 1e6:+.0f} ppm)")
        parts.append("playing" if f.get("run") else "not playing")
        parts.append(f"fifo {f.get('ff', 0)} B")
        parts.append(f"ring min {f.get('rmin', 0)}")
        parts.append(f"loop {f.get('lp', 0) / 1000:.2f} ms")
        if st["trim"] is not None:
            parts.append(f"trim {st['trim']:+d} ppm {'locked' if st['locked'] else 'unlocked'}")
        parts.append(fmt_volume(st["volume"]))
        if self.silent_since is not None:
            parts.append(f"silence {fmt_duration(now - self.silent_since)}")
        self.out("status: " + ", ".join(parts))

    # ---- loop -----------------------------------------------------------

    def step(self):
        now = time.monotonic()
        st, se, raw = self.read()
        self.device_change(st, raw)
        if st is None:
            self.prev, self.sess = None, None
            return
        self.reboot_events(st, raw)
        self.stream_events(st, se, now)
        self.input_events(se, now)
        self.silence_events(st, se, now)
        self.track_frames(st, se, now)
        self.status_line(st, se, now)
        self.prev, self.sess = st, se

    def summary(self):
        now = time.monotonic()
        total = self.streaming_total + (now - self.open_since if self.open_since else 0.0)
        self.out(f"summary: {self.streams_total} stream starts, {fmt_duration(total)} streaming")

    def run(self):
        self.out("monitoring usb2spdif (Ctrl+C to stop)")
        try:
            while True:
                self.step()
                time.sleep(self.args.interval)
        except KeyboardInterrupt:
            self.summary()
        finally:
            if self.log:
                self.log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--interval", type=float, default=0.25, metavar="S",
                    help="poll interval in seconds (default 0.25)")
    ap.add_argument("--silence", type=float, default=0.5, metavar="S",
                    help="all-zero samples for this long count as digital silence (default 0.5)")
    ap.add_argument("--status", type=float, default=0.0, metavar="S",
                    help="print a status line every S seconds while a stream is open")
    ap.add_argument("--log", metavar="FILE", help="append every line to this file")
    args = ap.parse_args()
    if args.interval <= 0 or args.silence < 0 or args.status < 0:
        sys.exit("arguments must be positive")
    Monitor(args).run()


if __name__ == "__main__":
    main()
