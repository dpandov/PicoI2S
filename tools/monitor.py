"""Live monitor of the usb2spdif bridge: sessions, format, data rate, silence.

Polls the device's diagnostic string (tools/diag.py) and its session string
a few times per second and prints one timestamped line per event:

    device connected / disconnected / rebooted (with the crash record)
    stream opened (format and PCM data rate) / closed / format changed
    playing (stream samples going out) / stalled / underrun
    S/PDIF output on / off (3 s after the audio ended)
    digital silence (all-zero samples for --silence seconds) / signal
    volume changes, clock lock (UAC1), counter increases (ov, lt)

With --status N a status line is printed every N seconds while a stream is
open: format, data rate, the sample rate measured from the device's frame
counter against the PC clock, FIFO level, ring minimum, longest main-loop
iteration, trim and the current silence.

    pip install pyusb libusb-package
    python tools/monitor.py                      # until Ctrl+C
    python tools/monitor.py --status 10 --log session.log
    python tools/monitor.py --silence 2 --interval 0.5

Needs the firmware's session string (index 0x23). Short events between two
polls (a stream opened and closed within --interval) are not seen.
"""
import argparse
import sys
import time

import libusb_package

from diag import STRID_DIAG, STRID_SESSION, parse_session, parse_status, read_diag

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


def fmt_volume(v):
    if v is None:
        return "?"
    return "muted" if v == "M" else ("0 dB" if v == 0 else f"-{v} dB")


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
            self.out(f"stream opened: {fmt_format(st['rate'], st['bits'])}")
            self.open_since = now
            self.streams_total += 1
            self.silent_since = None
        elif is_open and (fmt_changed or starts > 0):
            self.out(f"stream restarted: {fmt_format(st['rate'], st['bits'])}"
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
            if is_open:
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

        if p is not None and st["volume"] != p["volume"]:
            self.out(f"volume {fmt_volume(st['volume'])}")

        if st["uac1"] and p is not None and st["locked"] != p["locked"] and run:
            if st["locked"]:
                self.out(f"clock locked, trim {st['trim']:+d} ppm")
            else:
                self.out(f"clock unlocked, trim {st['trim']:+d} ppm")

    def silence_events(self, st, se, now):
        if se is None or st["link"] != "S" or not st["fields"].get("run", 0):
            return
        age = se["signal_age_ms"]
        silent = age is None or age >= self.args.silence * 1000.0
        if silent and self.silent_since is None:
            # The silence began when the last non-zero sample went out, or
            # with this stream if that was before it opened.
            began = now - (age / 1000.0 if age is not None else 0.0)
            from_open = self.open_since is not None and began <= self.open_since + self.args.interval
            if from_open:
                began = self.open_since
            self.silent_since = began
            self.out("digital silence" + (" since the stream opened" if from_open else ""))
        elif not silent and self.silent_since is not None:
            self.out(f"signal after {fmt_duration(now - self.silent_since)} of silence")
            self.silent_since = None

    def reboot_events(self, st, raw):
        if self.prev is not None and st["boots"] != self.prev["boots"]:
            self.out(f"device rebooted: boot {st['boots']}, previous ended in {st['prev']}"
                     + ("" if st["prev"] == "none" else f" ({raw.split(' | ')[0]})"))
            self.close_stream_state()

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
        if not self.args.status or st["link"] != "S":
            return
        if now - self.last_status_t < self.args.status:
            return
        self.last_status_t = now
        f = st["fields"]
        rate = self.measured_rate()
        parts = [fmt_format(st["rate"], st["bits"])]
        if rate is not None:
            parts.append(f"measured {rate:.1f} Hz ({(rate / st['rate'] - 1) * 1e6:+.0f} ppm)")
        parts.append("playing" if f.get("run") else "not playing")
        parts.append(f"fifo {f.get('ff', 0)} B")
        parts.append(f"ring min {f.get('rmin', 0)}")
        parts.append(f"loop {f.get('lp', 0) / 1000:.2f} ms")
        if st["uac1"]:
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
