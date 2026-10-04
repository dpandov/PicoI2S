# usb2spdif

USB to S/PDIF (Toslink) bridge firmware for the RP2350: the Waveshare
RP2350-One (default board) or the Raspberry Pi Pico 2.

The Pico enumerates as a USB Audio Class 2.0 device in asynchronous mode and
transmits the received PCM stream as S/PDIF through a Toslink optical
transmitter. No external DAC, no external clock, no drivers on the host.
For hosts without UAC2 support (TVs, older Windows) a jumper switches it to
a USB Audio Class 1.0 mode (see [UAC1 mode](#uac1-mode)).

## Supported formats

| Mode | Sample rates | Bit depths | Channels |
|---|---|---|---|
| UAC2 (default) | 32, 44.1, 48, 88.2, 96 kHz | 16, 24 | 2 (stereo) |
| UAC1 (GP5 to GND) | 32, 44.1, 48 kHz | 16 | 2 (stereo) |

## Hardware

Minimum build: a Waveshare RP2350-One (or a Raspberry Pi Pico 2, see
below) and a Toslink transmitter. The reference part is the
Cliff FCR684205T (OTJ-5 optical transmitter jack), a 3-pin through-hole
module on 2.54 mm pitch that fits a breadboard. Other TTL-input Toslink
transmitters (TOTX1350, PLT133 and similar) work the same way. Note that
TORX / FCR...R parts are receivers and will not work.

FCR684205T key figures from the datasheet: Vcc 2.7 to 5.5 V, VIH 2.0 V min,
16 Mbps max NRZ, 5 mA typical. Our highest cell rate is 12.288 MHz at
96 kHz, inside the rated speed.

| FCR684205T pin | Function | RP2350-One pin |
|---|---|---|
| 1 | GND | GND |
| 2 | Vcc | 3V3 |
| 3 | Vin | GP12, `SPDIF_PIN` in source |

Run the module from 3V3, not VBUS. Its input is rated to Vcc + 0.5 V and the
board drives 3.3 V logic, so a 3.3 V supply keeps everything in one domain.
Fit a 0.1 uF capacitor between Vcc and GND within 7 mm of the module leads,
as the datasheet requires. The module is ESD sensitive.

GP12 is the last pin of the GP0..GP13 row, on the short side of the board;
GND and 3V3 are the second and third pin of the other row.

Pin 1 is the pin nearest the mounting peg side when the jack opening faces
you; check the datasheet drawing in case of doubt.

### Mode jumper

GP5 selects the USB audio class. It is read once at power-up with the
internal pull-up enabled; changing it later has no effect until the next
power-up or reset.

| GP5 | Mode |
|---|---|
| Open | UAC2 (default) |
| Jumper to GND | UAC1 |

On the RP2350-One GP5 is the sixth pin of the GP0..GP13 row; GND is at the
top of the other row. On the Pico 2 (pin 7) GND is next to it on pin 8, so
a single 2.54 mm jumper (or a switch) between them does it.

### Raspberry Pi Pico 2

The firmware also builds for the Raspberry Pi Pico 2 with
`-DUSB2SPDIF_BOARD=pico2` (same RP2350A, 4 MB flash and 12 MHz crystal;
`PICO_BOARD` stays `pico2` for both boards). There S/PDIF is on GPIO 28
(pin 34), chosen because GND (pin 33) and 3V3 OUT (pin 36) sit next to it
at the top of the board, the mode jumper GP5 (pin 7) has GND next to it on
pin 8, and the status LED is the onboard LED on GPIO 25.

### Status LED

The status LED (the RP2350-One's WS2812 RGB LED on GP16, lit dim red,
colour and brightness in `src/board_config.h`; the Pico 2's onboard LED on
GPIO 25) shows whether audio is playing:

| LED | Meaning |
|---|---|
| 2 short flashes | The host has just enumerated the device |
| On | The host is streaming audio to the device |
| Off | No audio: no host, host idle, or stream closed |

"Audio" means the host has the stream open and is sending samples. Digital
silence inside an open stream still counts as audio.

The UART console runs at 921600 baud, 8N1, on GPIO 0 (TX) and GPIO 1 (RX).

A coax output can be added later with two resistors and a capacitor; the
firmware is unchanged.

## Architecture

```
USB host --iso OUT--> TinyUSB FIFO --main loop--> PCM ring --DMA IRQ: BMC encode--> DMA --> PIO --> GPIO --> Toslink
         <--feedback EP-- (FIFO fill level)
```

- **USB**: UAC2, two alternate settings (16-bit and 24-bit stereo), sample
  rate selected through the clock source entity, master mute and volume
  through a Feature Unit (see [Volume](#volume)). Asynchronous mode with an
  explicit feedback endpoint; the Pico is the audio clock master.
- **Clocking**: the system PLL is switched between two settings so the PIO
  divider is an integer for each sample-rate family; the fractional divider
  is never used in UAC2 mode, because it stretches single cells by one
  system-clock cycle. The settings are the best an integer PLL can do from
  the 12 MHz crystal (exhaustive search): the 48k family is exact, the
  44.1k family cannot be within the VCO limits. What remains is the crystal
  tolerance and, for 44.1k, -63 ppm, both absorbed by the feedback endpoint.

  | Family | sys_clk | PLL fbdiv / postdiv | PIO divider | Error |
  |---|---|---|---|---|
  | 48k (32, 48, 96) | 61.44 MHz | 128 / 5x5 | 15 / 10 / 5 | 0 ppm |
  | 44.1k (44.1, 88.2) | 101.6 MHz | 127 / 5x3 | 18 / 9 | -63 ppm |

  UAC1 mode runs the faster settings 147.43 / 135.43 MHz instead (-186 /
  -344 ppm): its clock servo cancels the error anyway, and the higher clock
  gives the trim a finer step.

- **S/PDIF**: the CPU biphase-mark encodes each frame (preamble, 24-bit
  sample, V/U/C/P bits) into 128 line cells with a table lookup; a PIO state
  machine shifts the cells out at 128 x Fs. Two DMA channels ping-pong over
  two 96-frame buffers, and each completion interrupt re-encodes the finished
  buffer from a small PCM ring that the main loop fills from the USB FIFO.
  Channel status: consumer, PCM, copy permitted, rate and word length of the
  stream.
- **Idle behaviour**: when the host stops streaming, silence is transmitted
  for 3 seconds so the receiver stays locked across track changes and short
  pauses. After 3 seconds without audio the S/PDIF output stops (line held
  idle, no carrier), letting the receiver drop lock, auto-switch inputs or
  go to standby. It restarts when the host streams again.
- **Buffering and latency**: the USB receive FIFO is held at 6 packets
  (6 ms) at every format by the feedback (UAC2) or the clock servo (UAC1);
  it can hold 12 packets of the largest format, so there is room for a host
  that sends early or late by several milliseconds. Behind it a 192-frame
  ring and two 96-frame DMA buffers hold 2 to 4 blocks. From a sample's
  arrival to its bit cells on the line the worst case is about 10 ms at
  96 kHz, 14 ms at 48 kHz and 18 ms at 32 kHz (16 or 24 bit alike); a
  stream starts once the FIFO holds that much. The host's own buffering
  and the receiver's processing come on top.

## Volume

The device has a volume control (a USB audio Feature Unit with master mute
and volume) because iOS only offers its volume slider for USB audio devices
that have one; without it the slider is locked at maximum. Windows and
Android also use it when present.

- Range 0 to -64 dB in 1 dB steps, plus mute. The volume is applied
  digitally in the bridge, with integer arithmetic.
- At 0 dB (unmuted) the samples pass unchanged: the output stays
  bit-perfect, 16-bit streams go out as 16-bit words.
- Below 0 dB the samples are scaled and rounded to 24 bits, and the S/PDIF
  channel status then declares 24-bit words, also for 16-bit streams (an
  attenuated 16-bit sample has more than 16 significant bits). Volume
  changes never interrupt the output.
- Changes ramp towards the new level by 1 dB per chunk of about 2 ms, like
  the PCM2704, so a large jump, a mute or an unmute fades instead of
  stepping; a new stream starts at the current level.
- Mute fades to digital silence; the stream still counts as audio (LED on,
  output running).
- 0 dB, unmuted at power-up and on every connection; hosts then set their
  own level.
- The test-tone build (`-DUSB2SPDIF_TEST_TONE=ON`) ignores it.

In WASAPI exclusive mode Windows still applies the device's volume
control, so for bit-perfect playback set the device's Windows volume to
100 % and unmute it. The test tools do that before they play.

## UAC1 mode

Many TVs, set-top boxes, car head units and Windows before 10 (1703) only
drive USB Audio Class 1.0 devices, typically the PCM2704-style USB DACs.
With GP5 grounded at power-up the bridge presents itself like one of those:

- USB Audio Class 1.0, no interface association (a plain composite
  device), its own USB ID 1209:0002 and the name "USB to SPDIF" (UAC2:
  "USB to S/PDIF"). Windows binds its built-in `usbaudio.sys`.
- One streaming format: 16-bit stereo at 32, 44.1 or 48 kHz, set by the
  host through the endpoint's sampling-frequency control. A host that never
  sets the rate gets 48 kHz.
- Master mute and volume like the PCM2704, the same as in UAC2 (see
  [Volume](#volume)).
- Adaptive isochronous endpoint without a feedback endpoint. The host sends
  at its own clock and the bridge follows it: a control loop on the USB
  receive buffer level trims the S/PDIF rate in steps of parts per billion
  (by dithering the PIO clock divider's fraction), so the S/PDIF clock is
  the host's clock. At stream start it locks within about 1.5 s, then
  narrows its bandwidth over about a minute and holds the rate within
  about 1 ppm of the host's. The trim range is +-1500 ppm,
  which leaves at least +-1100 ppm for the host's clock offset after the
  PLL error is cancelled.
- The picotool reset interface and the diagnostic strings work as in UAC2
  (`--pid 0x0002` for picotool).

Limits compared with UAC2: 16 bit only, 48 kHz maximum, and the S/PDIF
clock carries the host's clock quality (drift, and a small dither of the
PIO divider) instead of the Pico's crystal. Use UAC2 whenever the host
supports it.

## Source layout

```
src/
  main.c            init and main loop
  board_config.h    pin assignments
  usb/              USB descriptors, UAC2 / UAC1 class, TinyUSB config
  app/              audio pipeline, adaptive clock servo (UAC1), status LED,
                    console / diagnostic status, test tone
  spdif/            S/PDIF encoder, PIO program, DMA driver
  clock/            system PLL per sample-rate family
  diag/             crash and hang diagnostics
  selftest/         optional S/PDIF output self-test (decoder on core 1)
tools/              host-side test and diagnostic scripts
lib/tinyusb/        TinyUSB (git submodule, pinned release)
```

## Toolchain

- Raspberry Pi Pico SDK 2.x (RP2350 support)
- TinyUSB 0.21.0, as the git submodule `lib/tinyusb` (used instead of the
  older copy bundled with the SDK)
- ARM GCC, CMake, Ninja, picotool: all installed by the
  [Raspberry Pi Pico VS Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
- Raspberry Pi Debug Probe (SWD + UART console) for debugging, since the
  Pico's own USB port is occupied by the audio device

## Building

In VS Code, open the folder and run "Import Project" from the Raspberry Pi
Pico extension; it installs the SDK and toolchain and generates the
`.vscode` settings. TinyUSB is a submodule, so clone with
`git clone --recursive`, or run `git submodule update --init` in an
existing clone. From the command line:

```
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2
cmake --build build
```

This builds for the RP2350-One; for the Pico 2 add `-DUSB2SPDIF_BOARD=pico2`
(the pin assignment; `PICO_BOARD` stays `pico2`, it is the same chip).

To check the wiring and a receiver without a USB host, build with
`-DUSB2SPDIF_TEST_TONE=ON`: the output then carries a 1 kHz tone at -18 dBFS
(48 kHz / 24-bit) from power-up.

## Flashing

First time: hold BOOT while plugging in the board, then copy
`build/usb2spdif.uf2` to the drive, or run `picotool load -x build/usb2spdif.uf2`.

After that the firmware exposes the picotool reset interface, so it can be
reflashed without touching the button:

```
picotool load -x build/usb2spdif.uf2 -f --vid 0x1209 --pid 0x0001
```

In UAC1 mode use `--pid 0x0002`.

## Testing on Windows

The device appears as an audio output named "SPDIF Interface (USB to S/PDIF)"
("USB to SPDIF" in UAC1 mode).
Windows may make it the default playback device when first plugged in.
The host tools detect the mode from the diagnostic string; in UAC1 mode they
test 32 / 44.1 / 48 kHz at 16 bit and report the adaptive clock's rate trim.

Host tools in `tools/` (`pip install -r tools/requirements.txt`):

- `tools/stream_test.py` plays a tone at every rate and bit depth in WASAPI
  exclusive mode and checks, through the device's diagnostic string, that the
  device runs the requested format with no underruns and the S/PDIF output at
  the stream rate, and that the output stops 3 s after playback ends.
  Like `tools/selftest.py`, it first sets the device's Windows volume to
  100 % and unmuted, and fails while the device reports an attenuation.
- `tools/diag.py` reads the diagnostic string: previous-boot crash record
  (panic, fault or hang location) and live stream status. This works without
  a debug probe or UART.
- `tools/monitor.py` logs sessions, formats, digital silence and the other
  events of a running bridge; see Monitoring below.

## Monitoring

`tools/monitor.py` watches a running bridge from the host (any OS with
libusb; it reads the hidden USB strings, no audio driver involved) and
prints one timestamped line per event:

```
16:54:45 device connected: UAC2, non-Apple host, boot 3
16:54:52 stream opened: 48000 Hz / 24-bit (2.304 Mbit/s)
16:54:52 playing
16:54:52 S/PDIF output on: 48000 Hz
16:54:52 digital silence since the stream opened
16:54:58 signal after 5.8 s of silence
16:55:04 digital silence
16:55:10 stream closed after 17.9 s
16:55:13 S/PDIF output off after 20.9 s (3 s without audio)
```

Events: device connected / disconnected / rebooted (with the crash record
of the previous boot), stream opened / restarted / closed with the format
and the PCM data rate, playing / stalled / underrun, S/PDIF output on and
off, digital silence (all-zero samples, after the volume, for `--silence`
seconds, default 0.5) and the return of signal, volume changes, clock lock
in UAC1 mode, overrun and late-interrupt counts. `--status N` adds a line
every N seconds while a stream is open with the sample rate measured from
the device's frame counter against the PC clock (over 30 s, to about
100 ppm: the USB read timing limits it), FIFO level, ring minimum,
main-loop time and trim; `--log FILE` appends everything to a file.

```
python -m pip install pyusb libusb-package
python tools/monitor.py                        # events only, until Ctrl+C
python tools/monitor.py --status 10 --log session.log
```

Options: `--interval S` poll period (default 0.25 s), `--silence S` seconds
of all-zero samples that count as digital silence (default 0.5), `--status S`
status line period, `--log FILE`. Ctrl+C prints a summary (stream starts,
time streaming). Only `pyusb` and `libusb-package` are needed; `python -m pip`
avoids a stray `pip` launcher that belongs to another Python installation.
Events shorter than the poll interval (a stream opened and closed within
0.25 s) are not seen.

The firmware serves the counters behind this as a second hidden USB string
(index 0x23: stream starts, frames sent, time since the last non-zero
sample, uptime); `tools/diag.py` documents both strings.

## Self-test

A self-test build checks the S/PDIF output bit-exactly without a receiver
or any extra hardware. Core 1 samples the S/PDIF pin with a spare PIO state
machine, decodes the biphase-mark stream and compares it with what the
transmitter should send: framing, parity, channel status, the measured
sample rate and, while the PC plays a test pattern, every sample. The result
is served as a hidden USB string next to the diagnostic string.

```
cmake -S . -B build-selftest -G Ninja -DPICO_BOARD=pico2 -DUSB2SPDIF_SELFTEST=ON
cmake --build build-selftest
picotool load -x build-selftest/usb2spdif.uf2 -f --vid 0x1209 --pid 0x0001
python tools/selftest.py
```

`tools/selftest.py` builds a small native WASAPI player on first use (with
the `ziglang` package from `tools/requirements.txt`; PortAudio's exclusive
mode is not bit-exact) and plays the pattern at all 5 rates x 2 bit depths,
in an order that switches clock families both ways and changes bit depth at
the same rate, then checks the 3-second stop. PASS means that during
playback every decoded window was clean and carried only pattern frames,
the channel status had the right rate and word length, the measured rate was
within 2000 ppm, no window failed in between reads, the device reported no
underruns, overruns or late interrupts, and it did not reboot.

For a long run on one format, `python tools/selftest.py --soak 60 --rate
96000 --bits 24` plays for 60 minutes and logs every glitch with a time
stamp; it passes only with none. `--only 44100/16` runs one sweep item,
`--status` prints the current reports.

Combined with `-DUSB2SPDIF_TEST_TONE=ON`, the self-test checks the tone's
framing, channel status and rate (no pattern). To test the optics and a real
receiver, use the test tone build and watch the receiver's lock indicator.
The self-test does not change the audio path; the normal build contains none
of it.

## Status

- Step 1, bring-up: done.
- Step 2, UAC2 device: done. Enumerates on Windows 11, all 5 rates x 2 bit
  depths stream correctly, feedback holds the buffer at its set point. Verified
  on iPad and iPhone.
- Step 3, S/PDIF output: done on the pin. All 5 rates x 2 bit depths are
  bit-exact on the S/PDIF line (checked by a test build that decodes its own
  output), including clock family switches and the 3-second stop. A
  receiver locks to the output through the FCR684205T; playback verified
  from Windows, iPad and an Android streamer (Eversolo T8).
- Clock accuracy: with the accurate PLL settings the output measures
  exactly 32 000 / 48 000 / 96 000 Hz and -63 ppm at 44.1 / 88.2 kHz
  (self-test decoder against the crystal), verified bit-exact with the
  CPU at 61.44 MHz (worst main-loop iteration about 0.6 ms).
- Robustness: a late S/PDIF interrupt (tested with interrupts masked up to
  10 ms) repeats the last block with valid framing and resyncs by itself;
  USB overruns and underruns are detected and counted; logging never
  blocks. Long soak: 4 h at 96 kHz / 24-bit in UAC2 mode and 4 h at
  44.1 kHz / 16-bit in UAC1 mode, bit-exact with zero glitches, no
  underruns, overruns or late interrupts (UAC1 trim within 2 ppm of
  steady, locked in every read).
- USB stack: TinyUSB 0.21. UAC2 re-verified on Windows (all formats
  bit-exact, rapid stream open/close cycles) and on iPad / iPhone (Apple
  feedback path).
- Volume control (step 8): implemented and host-tested (exact gain
  arithmetic, word-length switching, request handling in both modes);
  verified on Windows (both drivers), iPad/iPhone and an Android streamer
  (Eversolo T8) in both modes.
- UAC1 mode: verified on Windows 11 with the classic usbaudio driver: all
  3 rates bit-exact, the S/PDIF clock follows the host within a few ppm,
  5-minute soak with the trim steady to 1 ppm, pause/resume without a
  trim swing. TV testing pending.

Known limitations:

- Apple hosts (macOS, iPadOS, iOS) need the 3-byte 10.14 feedback format,
  which Windows rejects. The device recognises Apple hosts by their
  enumeration order and serves them a matching configuration. macOS is
  untested.
- Linux is untested.
- The USB IDs 1209:0001 (UAC2) and 1209:0002 (UAC1) are pid.codes test
  IDs. Dedicated PIDs are needed before public release.
- In UAC1 mode on Windows before 8.1 the picotool reset interface has no
  driver (MS OS 2.0 descriptors need 8.1 or later) and shows up as an
  unknown device; audio is not affected.

## License

MIT. See `LICENSE`. Copy it, modify it, build on it, sell it, no permission
needed. The firmware links against the Raspberry Pi Pico SDK (BSD-3-Clause)
and TinyUSB (MIT), whose notices apply to those components.
