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
module on 2.54 mm pitch that fits a breadboard.

FCR684205T key figures from the datasheet: Vcc 2.7 to 5.5 V, VIH 2.0 V min,
16 Mbps max NRZ, 5 mA typical. Our highest cell rate is 12.288 MHz at
96 kHz, inside the rated speed.

| FCR684205T pin | Function | RP2350-One pin | Pico 2 pin |
|---|---|---|---|
| 1 | GND | GND | 33 (GND) |
| 2 | Vcc | 3V3 | 36 (3V3 OUT) |
| 3 | Vin | GP12 | 34 (GP28) |

The data pin is `SPDIF_PIN` in `src/board_config.h`.

Run the module from 3V3, not VBUS. Its input is rated to Vcc + 0.5 V and the
board drives 3.3 V logic, so a 3.3 V supply keeps everything in one domain.
Fit a 0.1 uF capacitor between Vcc and GND within 7 mm of the module leads,
as the datasheet requires. The module is ESD sensitive.

On the RP2350-One GP12 is the last pin of the GP0..GP13 row, on the short
side of the board; GND and 3V3 are the second and third pin of the other
row. On the Pico 2 the three are close together near the USB end of the
board: GND on pin 33, GP28 on 34, 3V3 OUT on 36.

On the FCR684205T, pin 1 is the pin nearest the mounting peg side when the
jack opening faces you; check the datasheet drawing in case of doubt.

### Mode jumper

With every jumper open (no jumpers fitted) the bridge runs as UAC2 with an
I2S output and no digital filter.

GP5 selects the USB audio class. It is read once at power-up with the
internal pull-up enabled; changing it later has no effect until the next
power-up or reset.

| GP5 | Mode | RP2350-One | Pico 2 |
|---|---|---|---|
| Open | UAC2 (default) | nothing fitted | nothing fitted |
| Jumper to GND | UAC1 | GP5 (sixth pin of the GP0..GP13 row) to GND (top of the other row) | pin 7 (GP5) to pin 8 (GND) |

On the Pico 2 GND is right next to GP5, so a single 2.54 mm jumper (or a
switch) does it.

### I2S output

Besides S/PDIF the bridge drives an I2S DAC (or, in the dual-line modes, a
pair of classic multibit DACs) with the same samples at the same time. The
I2S state machine sits on the same PIO block as the S/PDIF one, is started
in the same write and consumes one frame per the same number of
system-clock cycles, so the two outputs stay sample-aligned and follow the
UAC1 clock trim together. There is no MCLK. Clocks and data are low while
nothing plays (the outputs stop 3 s after the audio).

| Signal | RP2350-One | Pico 2 | Note |
|---|---|---|---|
| DATA (DL in the dual-line modes) | GP14 | GP16 (pin 21) | |
| DR | GP15 | GP17 (pin 22) | dual-line modes only |
| BCLK | GP26 | GP18 (pin 24) | |
| LRCK (LATCH in the dual-line modes) | GP27 | GP19 (pin 25) | |
| DEM | GP28 | GP20 (pin 26) | TDA1541A DEM clock, dual-line 16 only; low otherwise |
| GND | top of the same row | pins 23, 28 | |

On the RP2350-One the five signals sit in the row that starts with 5V, GND
and 3V3, in this order along the board: GP28, GP27, GP26, GP15, GP14.

The format is set by three jumpers on GPIO 6, 7 and 8 (Pico 2: pins 9,
10, 11, GND on pins 8 and 13), read once at power-up like the mode jumper;
a grounded pin is a 1:

| GP8 | GP7 | GP6 | Format |
|---|---|---|---|
| open | open | open | I2S (Philips), 32-bit slots, MSB one BCLK after the LRCK edge |
| open | open | gnd | Left-justified |
| open | gnd | open | Right-justified, 24-bit |
| open | gnd | gnd | Right-justified, 16-bit |
| gnd | open | open | Dual-line, 16-bit (TDA1541A simultaneous mode) |
| gnd | open | gnd | Dual-line, 18-bit (AD1865) |
| gnd | gnd | open | Dual-line, 20-bit (PCM63, PCM1702 pairs) |
| gnd | gnd | gnd | Dual-line, 24-bit |

Three more jumpers, GPIO 2, 3 and 4 (Pico 2: pins 4, 5, 6, GND on pins 3
and 8), select the filter of the dual-line formats (see
[Filters](#filters)), read the same way:

| GP2 | GP3 | GP4 | Dual-line filter |
|---|---|---|---|
| open | open | open | NOS: no filter, the samples as received, at the sample rate (default) |
| open | open | gnd | Sharp, linear phase, oversampled |
| open | gnd | open | Slow roll-off, linear phase, oversampled |
| open | gnd | gnd | NOS with sin(x)/x compensation, at the sample rate |
| gnd | open | open | Minimum phase (the slow magnitude), oversampled |

The remaining codes (GP2 grounded together with GP3 or GP4) are reserved
and select the sharp filter. I2S, left- and right-justified are never
filtered: they always run at the sample rate (64 BCLK per frame, two
32-bit slots, LRCK = Fs), whatever the filter jumpers say.

On the RP2350-One all seven jumper pins, GP2 to GP8, are in the GP0..GP13
row, on the side opposite the I2S pins (GP2 is its third pin, GP8 its
ninth). That row has no GND: use the GND at the top of the other row or
the pad on the underside.

#### Dual-line modes

Left and right data on their own pins, MSB first, right-justified to the
DAC's word length, common BCLK, and a LATCH pulse on the LRCK pin, timed
per the datasheets. The word length is the DAC's; the bridge sends the top
bits of its samples (truncated, no dither; the volume control is applied
before). Tie the DAC's left and right latch inputs together where it has
two (AD1865 LL and LR).

- AD1865, PCM63, PCM1702 (two's complement): the LSB sits in the last
  BCLK slot of the frame; LATCH is high through that slot (one BCLK
  period) and falls half a period after the rising edge that clocked the
  LSB, so the LSB is the last bit in before the latch. Leading slots carry
  zeros (the AD1865 keeps the last 18 bits before LATCH; the PCM63 /
  PCM1702 accept leading bits).
- TDA1541A in simultaneous mode (offset binary, no clock edge allowed
  while LE is high): 16 data slots per frame, the LSB in the last one; the
  LE pulse fills the BCLK-low half of the first slot of the next frame, so
  it rises with the falling edge after the LSB and falls with the rising
  edge that clocks the next MSB. BCLK stays at or below 6.144 MHz,
  within the chip's 6.4 MHz limit.
- DEM clock (dual-line 16 only): a 50 % square wave at the oversampled
  word rate with every filter, NOS included: 256 / 352.8 / 384 kHz at
  32 / 44.1 / 48 kHz and 352.8 / 384 kHz at 88.2 / 96 kHz. With NOS and
  the sinc compensation, which run at the sample rate, that is BCLK / 2
  (BCLK / 4 at 88.2 / 96 kHz), its edges on BCLK's falling edges. It is
  phase-locked to LATCH and comes from a third PIO state machine on the
  same block with the same divider and trim. It is meant for the
  TDA1541A's DEM oscillator pin (pin 16), in place of the capacitor. The
  pin is low in every other format.

#### Filters

The dual-line formats go through the filter selected by the jumpers
before they leave; every filter but NOS runs on core 1. The oversampling
filters interpolate
by 8 at 32 / 44.1 / 48 kHz and by 4 at 88.2 / 96 kHz, so the DAC runs at
256 / 352.8 / 384 kHz and its images move far outside the audio band,
where a simple analogue filter removes them. Fs is the sample rate:

| Filter | Response | Group delay (input samples) |
|---|---|---|
| Sharp | Linear phase. 8x: flat to 0.4535 Fs (20.0 kHz at 44.1 kHz, 21.8 kHz at 48 kHz), ripple below 0.0002 dB, 100 dB down from 0.5465 Fs. 4x: flat to 0.35 Fs (33.6 kHz at 96 kHz), 100 dB down from 0.65 Fs | 36 (8x), 13 (4x) |
| Slow | Linear phase. Flat to 0.25 Fs, -2.8 dB at 0.4535 Fs (20 kHz at 44.1 kHz), -3 dB near 0.46 Fs, 100 dB down from 0.75 Fs. Between Fs/2 and 0.75 Fs part of the first image gets through, by design; all other images are at least 100 dB down | 13 (8x), 8 (4x) |
| Minimum phase | The slow filter's magnitude exactly, with minimum phase: no pre-ringing | about 9 (8x), 4 (4x) |
| NOS | None: the samples as received, at the sample rate | 0 |
| Sinc compensation | At the sample rate: a short linear-phase FIR that, together with the DAC's zero-order hold, is flat at -4.0 dB (+-0.03 dB) up to 0.4535 Fs. The 4 dB of headroom keep its boost of up to +3.9 dB near Fs/2 from clipping. The images stay, as with NOS | 7 |

The filters use Q30 coefficients (`tools/i2s_filter_design.py`, which
also checks the figures above) and 64-bit accumulation. Intersample peaks
above full scale saturate, then the word is truncated to the DAC's length
(no dither). On top of the group delay the filtered output lags the
S/PDIF one by one 96-frame block; S/PDIF itself is never filtered or
delayed and carries the samples unchanged at the sample rate. NOS runs
without core 1 and adds no delay.

`tools/i2s_filter_plot.py` plots them: frequency, impulse and step
response at any sample rate (see [tools/README.md](tools/README.md)).

BCLK is the number of slots per frame times the output word rate. Slots
per frame: 16 for dual-line 16; for the other dual-line formats the
smallest count at or above the word length that divides the clock cycles
per frame (18 to 25); 64 for I2S / LJ / RJ. In UAC2 mode, with an oversampling
filter:

| Format | 32 kHz | 44.1 kHz | 48 kHz | 88.2 kHz | 96 kHz |
|---|---|---|---|---|---|
| Output word rate (dual-line, oversampled) | 256 kHz | 352.8 kHz | 384 kHz | 352.8 kHz | 384 kHz |
| Dual-line 16 (16 slots) | 4.096 MHz | 5.645 MHz | 6.144 MHz | 5.645 MHz | 6.144 MHz |
| Dual-line 18 | 5.12 MHz (20) | 6.35 MHz (18) | 7.68 MHz (20) | 6.35 MHz (18) | 9.216 MHz (24) |
| Dual-line 20 | 5.12 MHz (20) | 8.467 MHz (24) | 7.68 MHz (20) | 8.467 MHz (24) | 9.216 MHz (24) |
| Dual-line 24 | 6.4 MHz (25) | 8.467 MHz (24) | 9.6 MHz (25) | 8.467 MHz (24) | 9.216 MHz (24) |
| I2S / LJ / RJ (64 slots) | 2.048 MHz | 2.822 MHz | 3.072 MHz | 5.645 MHz | 6.144 MHz |

The slot counts in parentheses follow from the system clock, so they
differ in UAC1 mode: at 32 kHz dual-line 18 uses 18 slots (4.608 MHz),
dual-line 20 and 24 use 24 (6.144 MHz); at every other rate all three
use 24 (8.467 MHz in the 44.1 kHz family, 9.216 MHz in the 48 kHz one). With NOS and the sinc
compensation the slot counts stay and the output word rate is the sample
rate, so BCLK is 8 or 4 times lower (dual-line 16: 16 x Fs).

Limitations: the filters and the DEM clock exist for the dual-line formats
only. In UAC1 mode the clock trim that follows the host's rate is a
fractional PIO divider, dithered: the I2S clocks carry a phase error of up
to one system-clock cycle (about 7 ns) as a slow sawtooth. Asynchronous
DACs (ESS type) do not care; a synchronous DAC sees it as low-level
jitter. UAC2 mode has none of it. The 44.1 and 48 kHz settings run the
chip at 152.4 / 153.6 MHz, 1.6 / 2.4 % above its 150 MHz rating, to give
the filters their cycles without a larger clock error (see Clocking
below).

### Raspberry Pi Pico 2

The firmware also builds for the Raspberry Pi Pico 2 with
`-DUSB2SPDIF_BOARD=pico2` (same RP2350A, 4 MB flash and 12 MHz crystal;
`PICO_BOARD` stays `pico2` for both boards). There S/PDIF is on GPIO 28
(pin 34), chosen because GND (pin 33) and 3V3 OUT (pin 36) sit next to it
at the top of the board, the mode jumper GP5 (pin 7) has GND next to it on
pin 8, and the status LED is the onboard LED on GPIO 25; the I2S pins and the
format and filter jumpers (pins 9 to 11 and 4 to 6) are in the tables
above.

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
- **Clocking**: the system PLL is switched per sample rate so the PIO
  divider is an integer; the fractional divider is never used in UAC2
  mode, because it stretches single cells by one system-clock cycle. The
  settings are the best an integer PLL can do from the 12 MHz crystal
  (exhaustive search) among the clocks fast enough for the dual-line
  filters on core 1 (see [I2S output](#i2s-output)): 32 and 48 kHz are
  exact; 44.1 / 88.2 kHz cannot be within the VCO limits; 96 kHz has one
  exact clock, 61.44 MHz, which is too slow for the filters. What remains
  is the crystal tolerance and the errors below, absorbed by the feedback
  endpoint.

  | Rate | sys_clk | PLL fbdiv / postdiv | PIO divider | Error |
  |---|---|---|---|---|
  | 32 kHz | 102.4 MHz | 128 / 5x3 | 25 | 0 ppm |
  | 44.1 kHz | 152.4 MHz | 127 / 5x2 | 27 | -63 ppm |
  | 48 kHz | 153.6 MHz | 128 / 5x2 | 25 | 0 ppm |
  | 88.2 kHz | 101.6 MHz | 127 / 5x3 | 9 | -63 ppm |
  | 96 kHz | 147.43 MHz | 86 / 7x1 | 12 | -186 ppm |

  152.4 and 153.6 MHz are 1.6 and 2.4 % above the RP2350's 150 MHz
  rating. UAC1 mode runs 147.43 / 135.43 MHz for the 48k / 44.1k
  families instead (-186 / -344 ppm): its clock servo cancels the error
  anyway, and the higher clock gives the trim a finer step.

- **S/PDIF**: the CPU biphase-mark encodes each frame (preamble, 24-bit
  sample, V/U/C/P bits) into 128 line cells with a table lookup; a PIO state
  machine shifts the cells out at 128 x Fs. Two DMA channels ping-pong over
  two 96-frame buffers, and each completion interrupt re-encodes the finished
  buffer from a small PCM ring that the main loop fills from the USB FIFO.
  Channel status: consumer, PCM, copy permitted, rate and word length of the
  stream.
- **I2S / dual-line**: a second state machine on the same PIO block, started
  in the same write and fed with every block the S/PDIF interrupt fetches,
  so both outputs stay sample-aligned. The dual-line formats pass through
  the selected filter on core 1 first, NOS excepted (see
  [Filters](#filters)); the S/PDIF output is unaffected.
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
  i2s/              I2S / dual-line output: program generator, packer,
                    dual-line filters (core 1), TDA1541A DEM clock
  clock/            system PLL per sample rate
  diag/             crash and hang diagnostics
  selftest/         optional S/PDIF and I2S self-test (decoders on core 1)
tools/              host-side test and diagnostic scripts (manual: tools/README.md)
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

Host tools in `tools/` (`python -m pip install -r tools/requirements.txt`),
documented in [tools/README.md](tools/README.md):

- `tools/selftest.py`: bit-exact check of both outputs with the self-test
  firmware (see Self-test below).
- `tools/stream_test.py`: a tone at every rate and bit depth, checked
  through the diagnostic string, with the normal firmware.
- `tools/monitor.py`: live event log (see Monitoring below).
- `tools/diag.py`: crash record of the previous boot and live status,
  without a debug probe or UART.
- `tools/i2s_filter_plot.py`, `tools/i2s_filter_design.py`: the dual-line
  filters.

## Monitoring

`tools/monitor.py` watches a running bridge from the host (any OS with
libusb; it reads hidden USB strings, no audio driver involved) and prints
one timestamped line per event:

```
16:54:45 device connected: UAC2, non-Apple host, boot 3
16:54:45   config: board Waveshare RP2350-One, firmware 1.03, normal build
16:54:45   config: USB UAC2 (mode jumper GP5 open)
16:54:45   config: output format dual-line 16-bit (TDA1541A simultaneous mode), DEM clock on (jumpers GP8 GP7 GP6 = 100)
16:54:45   config: filter sharp, linear phase, 8x at 32 / 44.1 / 48 kHz, 4x at 88.2 / 96 kHz (jumpers GP2 GP3 GP4 = 001)
16:54:52 stream opened: 48000 Hz / 24-bit (2.304 Mbit/s), volume 0 dB
16:54:52 playing
16:54:52 S/PDIF output on: 48000 Hz
16:54:52 digital silence since the stream opened
16:54:58 signal after 5.8 s of silence
16:55:04 digital silence
16:55:10 stream closed after 17.9 s
16:55:13 S/PDIF output off after 20.9 s (3 s without audio)
```

It also reports underruns, volume changes, the UAC1 clock lock, overrun,
late-interrupt and late I2S block counts, and on request a periodic status
line with the sample rate measured against the PC clock. The firmware
serves the counters behind it as a second hidden USB string (index 0x23)
next to the diagnostic string, and the configuration it booted with
(board, firmware, build, USB mode, output format, filter, jumpers) as a
third (index 0x24). Events, options and the strings are described in
[tools/README.md](tools/README.md).

## Self-test

A self-test build checks the S/PDIF and I2S outputs bit-exactly without a
receiver, a DAC or any extra hardware. Core 1 samples the S/PDIF pin with a
spare PIO state machine, decodes the biphase-mark stream and compares it
with what the transmitter should send: framing, parity, channel status, the
measured sample rate and, while the PC plays a test pattern, every sample.
A second state machine captures the I2S pins on every BCLK edge; the
decoder frames the stream per the jumper format (slot count, LSB and LATCH
position as the transmitter computes them) and checks framing and samples.
The direct formats and the dual-line formats with the NOS filter are
checked against the pattern; the dual-line formats with any other filter
(the sinc compensation included) are checked bit-exact against the
window's own S/PDIF samples run through the same filter on the device, so
no host-side model is involved. The results are served as two hidden USB
strings next to the diagnostic string (formats in
`src/selftest/selftest_report.h` and [tools/README.md](tools/README.md));
the I2S report names the format and the filter and counts late blocks.

```
cmake -S . -B build-selftest -G Ninja -DPICO_BOARD=pico2 -DUSB2SPDIF_SELFTEST=ON
cmake --build build-selftest
picotool load -x build-selftest/usb2spdif.uf2 -f --vid 0x1209 --pid 0x0001
python tools/selftest.py
```

`tools/selftest.py` plays a test pattern at all 5 rates x 2 bit depths
(bit-exact, with its own WASAPI player), checks every report and the
3-second stop, and prints PASS or FAIL per format for each output;
`--soak 60 --rate 96000 --bits 24` runs one format for an hour. Pass
criteria, options and the report fields are in
[tools/README.md](tools/README.md).

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
  output), including clock switches and the 3-second stop. A
  receiver locks to the output through the FCR684205T; playback verified
  from Windows, iPad and an Android streamer (Eversolo T8).
- Clock accuracy: with the accurate PLL settings the output measures
  exactly 32 000 / 48 000 Hz, -63 ppm at 44.1 / 88.2 kHz and -186 ppm at
  96 kHz (self-test decoder against the crystal), bit-exact at every rate
  and word length.
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
- I2S output (step 9): the I2S format is bit-exact on the board at all ten
  stream formats (self-test capture) alongside the S/PDIF sweep; the
  dual-line formats, the five filters and the DEM clock pass the host
  tests (the filters bit-exact against their C reference) and the
  cycle-accurate PIO simulation for every format and clock. Checks on the
  board with the format and filter jumpers, UAC1 with the I2S output, and
  DAC tests are pending. No listening tests yet.

Known limitations:

- Apple hosts (macOS, iPadOS, iOS) need the 3-byte 10.14 feedback format,
  which Windows rejects. The device recognises Apple hosts by their
  enumeration order and serves them a matching configuration. macOS is
  untested.
- Linux is untested.
- In UAC2 mode the 44.1 and 48 kHz settings run the RP2350 at 152.4 /
  153.6 MHz, 1.6 / 2.4 % above its 150 MHz rating (see Clocking).
- The USB IDs 1209:0001 (UAC2) and 1209:0002 (UAC1) are pid.codes test
  IDs. Dedicated PIDs are needed before public release.
- In UAC1 mode on Windows before 8.1 the picotool reset interface has no
  driver (MS OS 2.0 descriptors need 8.1 or later) and shows up as an
  unknown device; audio is not affected.

## License

MIT. See `LICENSE`. Copy it, modify it, build on it, sell it, no permission
needed. The firmware links against the Raspberry Pi Pico SDK (BSD-3-Clause)
and TinyUSB (MIT), whose notices apply to those components.
