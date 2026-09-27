# usb2spdif

USB to S/PDIF (Toslink) bridge firmware for the Raspberry Pi Pico 2 (RP2350).

The Pico enumerates as a USB Audio Class 2.0 device in asynchronous mode and
transmits the received PCM stream as S/PDIF through a Toslink optical
transmitter. No external DAC, no external clock, no drivers on the host.

## Supported formats

| Sample rates | Bit depths | Channels |
|---|---|---|
| 32, 44.1, 48, 88.2, 96 kHz | 16, 24 | 2 (stereo) |

## Hardware

Minimum build: a Pico 2 and a Toslink transmitter. The reference part is the
Cliff FCR684205T (OTJ-5 optical transmitter jack), a 3-pin through-hole
module on 2.54 mm pitch that fits a breadboard. Other TTL-input Toslink
transmitters (TOTX1350, PLT133 and similar) work the same way. Note that
TORX / FCR...R parts are receivers and will not work.

FCR684205T key figures from the datasheet: Vcc 2.7 to 5.5 V, VIH 2.0 V min,
16 Mbps max NRZ, 5 mA typical. Our highest cell rate is 12.288 MHz at
96 kHz, inside the rated speed.

| FCR684205T pin | Function | Pico 2 pin |
|---|---|---|
| 1 | GND | GND (pin 33) |
| 2 | Vcc | 3V3 OUT (pin 36) |
| 3 | Vin | GPIO 28 (pin 34), `SPDIF_PIN` in source |

Run the module from 3V3, not VBUS. Its input is rated to Vcc + 0.5 V and the
Pico drives 3.3 V logic, so a 3.3 V supply keeps everything in one domain.
Fit a 0.1 uF capacitor between Vcc and GND within 7 mm of the module leads,
as the datasheet requires. The module is ESD sensitive.

GPIO 28 was chosen because pins 33, 34 and 36 sit together at the top of
the board, keeping the three wires to the module short.

Pin 1 is the pin nearest the mounting peg side when the jack opening faces
you; check the datasheet drawing in case of doubt.

### Status LED

The onboard LED (GPIO 25) shows whether audio is playing:

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
  rate selected through the clock source entity. Asynchronous mode with an
  explicit feedback endpoint; the Pico is the audio clock master.
- **Clocking**: the system PLL is switched between two settings so the PIO
  divider is an integer for each sample-rate family. Residual error is
  absorbed by the feedback endpoint.

  | Family | sys_clk | PLL fbdiv / postdiv | Error |
  |---|---|---|---|
  | 48k (32, 48, 96) | 147.4286 MHz | 86 / 7x1 | -186 ppm |
  | 44.1k (44.1, 88.2) | 135.4286 MHz | 79 / 7x1 | -344 ppm |

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

## Source layout

```
src/
  main.c            init and main loop
  board_config.h    pin assignments
  usb/              USB descriptors, UAC2 class, TinyUSB config
  app/              audio pipeline, status LED, console / diagnostic status,
                    test tone
  spdif/            S/PDIF encoder, PIO program, DMA driver
  clock/            system PLL per sample-rate family
  diag/             crash and hang diagnostics
tools/              host-side test and diagnostic scripts
```

## Toolchain

- Raspberry Pi Pico SDK 2.x (RP2350 support, bundled TinyUSB)
- ARM GCC, CMake, Ninja, picotool: all installed by the
  [Raspberry Pi Pico VS Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
- Raspberry Pi Debug Probe (SWD + UART console) for debugging, since the
  Pico's own USB port is occupied by the audio device

## Building

In VS Code, open the folder and run "Import Project" from the Raspberry Pi
Pico extension; it installs the SDK and toolchain and generates the
`.vscode` settings. From the command line:

```
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2
cmake --build build
```

To check the wiring and a receiver without a USB host, build with
`-DUSB2SPDIF_TEST_TONE=ON`: the output then carries a 1 kHz tone at -18 dBFS
(48 kHz / 24-bit) from power-up.

## Flashing

First time: hold BOOTSEL while plugging in the Pico 2, then copy
`build/usb2spdif.uf2` to the drive, or run `picotool load -x build/usb2spdif.uf2`.

After that the firmware exposes the picotool reset interface, so it can be
reflashed without touching the button:

```
picotool load -x build/usb2spdif.uf2 -f --vid 0x1209 --pid 0x0001
```

## Testing on Windows

The device appears as an audio output named "SPDIF Interface (USB to S/PDIF)".
Windows may make it the default playback device when first plugged in.

Host tools in `tools/` (`pip install -r tools/requirements.txt`):

- `tools/stream_test.py` plays a tone at every rate and bit depth in WASAPI
  exclusive mode and checks, through the device's diagnostic string, that the
  device runs the requested format with no underruns and the S/PDIF output at
  the stream rate, and that the output stops 3 s after playback ends.
- `tools/diag.py` reads the diagnostic string: previous-boot crash record
  (panic, fault or hang location) and live stream status. This works without
  a debug probe or UART.

## Status

- Step 1, bring-up: done.
- Step 2, UAC2 device: done. Enumerates on Windows 11, all 5 rates x 2 bit
  depths stream correctly, feedback holds the buffer at half full. Verified
  on iPad and iPhone.
- Step 3, S/PDIF output: done on the pin. All 5 rates x 2 bit depths are
  bit-exact on the S/PDIF line (checked by a test build that decodes its own
  output), including clock family switches and the 3-second stop. A
  receiver locks to the output through the FCR684205T; playback verified
  from Windows and iPad.

Known limitations:

- Apple hosts (macOS, iPadOS, iOS) need the 3-byte 10.14 feedback format,
  which Windows rejects. The device recognises Apple hosts by their
  enumeration order and serves them a matching configuration. macOS is
  untested.
- Linux is untested.
- The USB ID 1209:0001 is the pid.codes test ID. A dedicated PID is needed
  before public release.

## License

MIT. See `LICENSE`. Copy it, modify it, build on it, sell it, no permission
needed. The firmware links against the Raspberry Pi Pico SDK (BSD-3-Clause)
and TinyUSB (MIT), whose notices apply to those components.
