"""Windows stream test: plays a 1 kHz tone at every supported rate and bit
depth in WASAPI exclusive mode and checks the device's own view of the
stream through the diagnostic string (see tools/diag.py).

A combination passes when, during playback, the device reports streaming at
the requested rate and resolution, the sink is running and there are no
underruns.

    pip install sounddevice numpy pyusb libusb-package
    python tools/stream_test.py
"""
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


def main():
    backend = libusb_package.get_libusb1_backend()
    out = find_output()
    all_ok = True
    for bits in BITS:
        for rate in RATES:
            # WASAPI streams must be opened from the main thread.
            sd.play(tone(rate, bits), samplerate=rate, device=out,
                    extra_settings=sd.WasapiSettings(exclusive=True), blocking=False)
            t0 = time.time()
            samples = []
            for at in SAMPLE_AT:
                time.sleep(max(0.0, at - (time.time() - t0)))
                samples.append(read_diag(backend).split("| ")[-1])
            sd.wait()
            want = f"{rate}/{bits}"
            ok = all(s.startswith("S") and s.split()[1] == want and " ur0 " in s
                     and " run1" in s and s.endswith("led1") for s in samples)
            all_ok &= ok
            print(f"{rate:6d} Hz {bits}-bit: {'PASS' if ok else 'FAIL'}   {samples[-1]}")
    print("ALL PASS" if all_ok else "SOME FAILED")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
