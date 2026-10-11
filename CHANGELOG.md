# Changelog

User-visible changes per release. The version is `project(PicoI2S
VERSION x.y)` in `CMakeLists.txt`, shown as x.0y and reported as the USB
device release; every build also carries a build number and commit
(`tools/diag.py`, `tools/monitor.py`, `picotool info`). Releases are tagged
`vx.yy` on `main`.

## [Unreleased]

## [1.04] - 2026-10-11

The first public release, still under the pid.codes test USB IDs
1209:0001 / 1209:0002 until dedicated IDs are assigned. Since 1.0: the
I2S and dual-line outputs with their filters, MCLK and the DEM clock, the
S/PDIF input, the keep-clocks option, version reporting.

### USB audio
- USB Audio Class 2.0, asynchronous with a feedback endpoint: 32 / 44.1 /
  48 / 88.2 / 96 kHz, 16 and 24 bit, stereo. Tested on Windows 11, Linux,
  iPad / iPhone and Android.
- UAC1 mode (GP5 grounded at boot) for hosts without UAC2: adaptive, the
  output clock follows the host; 32 / 44.1 / 48 kHz, 16 bit.
- Host volume and mute on the device, bit-exact at 0 dB, ramped 1 dB per
  2 ms.
- 6 ms receive FIFO set point.

### Outputs
- S/PDIF (Toslink) output with the same samples, unfiltered.
- I2S, left- and right-justified (16 or, with `PICOI2S_RJ24`, 24 bit)
  with a 256 Fs MCLK.
- Dual-line 16 / 18 / 20 / 24 bit for classic multibit DACs (PCM56,
  AD1865 / AD1860, AD1862 / PCM63 / PCM1702, PCM1704) and TDA1541A
  simultaneous mode with a DEM clock.
- Filters for the dual-line formats: sharp, slow and minimum-phase
  oversampling (8x at 44.1 / 48 kHz, 4x at 88.2 / 96 kHz), NOS, and NOS
  with sin(x)/x compensation.
- Output, format and filter set by jumpers, read at boot.
- The outputs stop 3 s after the audio; `PICOI2S_KEEP_CLOCKS` keeps the
  DAC clocks running with silence instead.

### S/PDIF input
- Coax and Toslink input, played when USB is idle; USB takes the output
  back on its first audible sample. Never resampled: the output clock
  follows the input.
- Pre-emphasis flag carried to the S/PDIF output and de-emphasis on the
  I2S / dual-line path.
- Fades on every switch between USB and the input.
- A rate change without a signal gap unlocks within about 4 ms, before
  any sample plays at the wrong rate.

### Boards and tools
- Waveshare RP2350-One (default) and Raspberry Pi Pico 2.
- Reflash without BOOTSEL through the picotool reset interface.
- Crash and hang records from the previous boot, live status, version and
  output state as USB strings; host tools `diag.py`, `monitor.py`,
  `selftest.py` (bit-exact check of both outputs with the self-test
  build), `stream_test.py` and the filter design scripts.
