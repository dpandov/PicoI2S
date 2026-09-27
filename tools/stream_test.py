"""Windows stream test: plays a 1 kHz tone at every supported rate and bit
depth in WASAPI exclusive mode and checks the device's own view of the
stream through the diagnostic string (see tools/diag.py).

A combination passes when, during playback, the device reports streaming at
the requested rate and resolution, S/PDIF output at the same rate, audio
playing, LED on and no underruns. The sweep alternates sample-rate families
and bit depths, so it covers clock switches and word-length changes on a
running output. After the last combination the output must keep sending
silence for about 3 s and then stop.

    pip install sounddevice numpy pyusb libusb-package
    python tools/stream_test.py
"""
import re
import sys
import time

import libusb_package
import numpy as np
import sounddevice as sd

from diag import read_diag

RATES = [32000, 44100, 48000, 88200, 96000]
BITS = [16, 24]
PLAY_SECONDS = 6
SAMPLE_AT = (2.0, 3.5, 5.0)
# Seconds after playback ends: output still running, then stopped.
IDLE_CHECK_AT = (2.0, 4.0)


def find_output():
    wasapi = next(i for i, h in enumerate(sd.query_hostapis()) if "WASAPI" in h["name"])
    for i, d in enumerate(sd.query_devices()):
        if d["hostapi"] == wasapi and "S/PDIF" in d["name"] and d["max_output_channels"] >= 2:
            return i
    sys.exit("usb2spdif WASAPI output not found")


def tone(rate, bits):
    t = np.arange(rate * PLAY_SECONDS) / rate
    full = 32767 if bits == 16 else 0x7FFFFF00   # 24-bit data in 32-bit containers
    x = (0.25 * full * np.sin(2 * np.pi * 1000 * t)).astype("int16" if bits == 16 else "int32")
    return np.column_stack([x, x])


def status_fields(status):
    """'So 48000/24 ff3104 ur0 run1 led1 tx48000' -> dict of fields."""
    tok = status.split()
    f = {"link": tok[0][:1] if tok else "", "format": tok[1] if len(tok) > 1 else ""}
    for t in tok[2:]:
        m = re.fullmatch(r"([a-z]+)(\d+)", t)
        if m:
            f[m.group(1)] = int(m.group(2))
    return f


def playing_ok(status, rate, bits):
    f = status_fields(status)
    return (f["link"] == "S" and f["format"] == f"{rate}/{bits}" and "ff" in f
            and f.get("ur") == 0 and f.get("run") == 1 and f.get("led") == 1
            and f.get("tx") == rate)


def sample_after(backend, t0, times):
    samples = []
    for at in times:
        time.sleep(max(0.0, at - (time.time() - t0)))
        samples.append(read_diag(backend).split("| ")[-1])
    return samples


def main():
    backend = libusb_package.get_libusb1_backend()
    out = find_output()
    all_ok = True
    # Rate outer, bits inner: every rate step changes the clock family
    # (32 -> 44.1 -> 48 -> 88.2 -> 96), every bits step keeps the rate.
    for rate in RATES:
        for bits in BITS:
            # WASAPI streams must be opened from the main thread.
            sd.play(tone(rate, bits), samplerate=rate, device=out,
                    extra_settings=sd.WasapiSettings(exclusive=True), blocking=False)
            samples = sample_after(backend, time.time(), SAMPLE_AT)
            sd.wait()
            ok = all(playing_ok(s, rate, bits) for s in samples)
            all_ok &= ok
            print(f"{rate:6d} Hz {bits}-bit: {'PASS' if ok else 'FAIL'}   {samples[-1]}")

    # 3 s of silence after the stream ends, then no carrier.
    running, stopped = sample_after(backend, time.time(), IDLE_CHECK_AT)
    fr, fs = status_fields(running), status_fields(stopped)
    ok = (fr.get("tx") == RATES[-1] and fr.get("led") == 0
          and fs.get("tx") == 0 and fs.get("led") == 0)
    all_ok &= ok
    print(f"idle stop:        {'PASS' if ok else 'FAIL'}   {running}  ->  {stopped}")
    print("ALL PASS" if all_ok else "SOME FAILED")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
