"""Windows stream test: plays a 1 kHz tone at every supported rate and bit
depth in WASAPI exclusive mode and checks the device's own view of the
stream through the diagnostic string (see tools/diag.py).

A combination passes when, during playback, the device reports streaming at
the requested rate and resolution, S/PDIF output at the same rate, audio
playing, LED on, and no underruns or overruns since the stream started. The
longest main-loop iteration, late S/PDIF interrupts and the lowest ring
level are reported. The sweep alternates sample-rate families
and bit depths, so it covers clock switches and word-length changes on a
running output. After the last combination the output must keep sending
silence for about 3 s and then stop.

In UAC1 mode (mode pin grounded, detected from the diagnostic string) the
device offers 32 / 44.1 / 48 kHz at 16 bit only, so only those are played;
the S/PDIF rate trim of the adaptive clock and the clock servo lock are
reported per format.

The device has a volume control that Windows applies in exclusive mode, so
the Windows volume of the device is first set to 100 % and unmuted with
tools/wasapi_play.exe --set-volume-max (built on first use, as by
tools/selftest.py; if that fails the test warns and continues). A
combination fails while the device reports an attenuation or mute
(diagnostic token v<dB> other than v0, or vM).

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
from selftest import set_volume_max

RATES = [32000, 44100, 48000, 88200, 96000]
BITS = [16, 24]
UAC1_RATES = [32000, 44100, 48000]
UAC1_BITS = [16]
PLAY_SECONDS = 6
SAMPLE_AT = (2.0, 3.5, 5.0)
# Seconds after playback ends: output still running, then stopped.
IDLE_CHECK_AT = (2.0, 4.0)


def find_output():
    wasapi = next(i for i, h in enumerate(sd.query_hostapis()) if "WASAPI" in h["name"])
    for i, d in enumerate(sd.query_devices()):
        if (d["hostapi"] == wasapi and d["max_output_channels"] >= 2 and
                ("Pico USB to I2S" in d["name"] or "Pico I2S" in d["name"])):
            return i
    sys.exit("PicoI2S WASAPI output not found")


def tone(rate, bits):
    t = np.arange(rate * PLAY_SECONDS) / rate
    full = 32767 if bits == 16 else 0x7FFFFF00   # 24-bit data in 32-bit containers
    x = (0.25 * full * np.sin(2 * np.pi * 1000 * t)).astype("int16" if bits == 16 else "int32")
    return np.column_stack([x, x])


def status_fields(status):
    """'So 48000/24 ff4656 ur0 run1 led1 tx48000 ov0 ... v0' -> dict of fields.
    UAC1 mode: second character '1', and 'tr<ppm>L|U' before the volume.
    Volume: 'volume' is the attenuation in dB or 'M' (muted), absent with
    firmware that has no volume control."""
    tok = status.split()
    f = {"link": tok[0][:1] if tok else "", "format": tok[1] if len(tok) > 1 else "",
         "uac1": tok[0][1:2] == "1" if tok else False}
    for t in tok[2:]:
        m = re.fullmatch(r"tr(-?\d+)([LU])", t)
        if m:
            f["trim"], f["locked"] = int(m.group(1)), m.group(2) == "L"
            continue
        m = re.fullmatch(r"v(\d+|M)", t)
        if m:
            f["volume"] = "M" if m.group(1) == "M" else int(m.group(1))
            continue
        m = re.fullmatch(r"([a-z]+)(\d+)", t)
        if m:
            f[m.group(1)] = int(m.group(2))
    return f


def playing_ok(status, rate, bits):
    f = status_fields(status)
    return (f["link"] == "S" and f["format"] == f"{rate}/{bits}" and "ff" in f
            and f.get("ur") == 0 and f.get("ov") == 0 and f.get("run") == 1
            and f.get("led") == 1 and f.get("tx") == rate and f.get("volume", 0) == 0)


def volume_note(status):
    vol = status_fields(status).get("volume", 0)
    if vol == "M":
        return "  (device muted: unmute it in Windows)"
    if vol:
        return f"  (device volume -{vol} dB: set the Windows volume to 100 %)"
    return ""


def sample_after(backend, t0, times):
    samples = []
    for at in times:
        time.sleep(max(0.0, at - (time.time() - t0)))
        samples.append(read_diag(backend).split("| ")[-1])
    return samples


def main():
    backend = libusb_package.get_libusb1_backend()
    status = read_diag(backend).split("| ")[-1]
    if status.startswith("<"):
        sys.exit(f"PicoI2S not found: {status}")
    uac1 = status_fields(status)["uac1"]
    rates, bits_list = (UAC1_RATES, UAC1_BITS) if uac1 else (RATES, BITS)
    print(f"device in {'UAC1' if uac1 else 'UAC2'} mode: {status}")
    out = find_output()
    set_volume_max()
    all_ok = True
    lp_max = 0
    # Rate outer, bits inner: every rate step changes the clock family
    # (32 -> 44.1 -> 48 -> 88.2 -> 96), every bits step keeps the rate.
    for rate in rates:
        for bits in bits_list:
            # WASAPI streams must be opened from the main thread.
            sd.play(tone(rate, bits), samplerate=rate, device=out,
                    extra_settings=sd.WasapiSettings(exclusive=True), blocking=False)
            samples = sample_after(backend, time.time(), SAMPLE_AT)
            sd.wait()
            ok = all(playing_ok(s, rate, bits) for s in samples)
            all_ok &= ok
            fields = [status_fields(s) for s in samples]
            lp = max(f.get("lp", 0) for f in fields)
            lt = max(f.get("lt", 0) for f in fields)
            rmin = min(f.get("rmin", 0) for f in fields)
            lp_max = max(lp_max, lp)
            clock = ""
            if uac1:
                last = fields[-1]
                clock = (f", trim {last.get('trim')} ppm "
                         f"{'locked' if last.get('locked') else 'not locked'}")
            print(f"{rate:6d} Hz {bits}-bit: {'PASS' if ok else 'FAIL'}   "
                  f"lp {lp} us, lt {lt}, rmin {rmin}{clock}   {samples[-1]}"
                  f"{volume_note(samples[-1])}")

    # 3 s of silence after the stream ends, then no carrier.
    running, stopped = sample_after(backend, time.time(), IDLE_CHECK_AT)
    fr, fs = status_fields(running), status_fields(stopped)
    ok = (fr.get("tx") == rates[-1] and fr.get("led") == 0
          and fs.get("tx") == 0 and fs.get("led") == 0)
    all_ok &= ok
    print(f"idle stop:        {'PASS' if ok else 'FAIL'}   {running}  ->  {stopped}")
    print(f"longest main-loop iteration while playing: {lp_max} us")
    print("ALL PASS" if all_ok else "SOME FAILED")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
