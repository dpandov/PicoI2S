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

The onboard LED (GPIO 25) is the LOCK indicator:

| LED | Meaning |
|---|---|
| Off | USB not enumerated, or idle (silence being transmitted) |
| Blinking | Enumerated, no audio stream open |
| On | Host is streaming and the feedback loop has settled |

A coax output can be added later with two resistors and a capacitor; the
firmware is unchanged.

## Architecture

```
USB host --iso OUT--> TinyUSB UAC2 --FIFO--> subframe builder --DMA--> PIO (BMC) --> GPIO --> Toslink
              <--feedback EP--   (buffer fill level)
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

- **S/PDIF**: a PIO state machine performs biphase-mark encoding at 128 x Fs
  cells per second. The CPU builds 32-bit subframes (preamble, 24-bit
  left-justified sample, V/U/C/P bits) into a double buffer that DMA streams
  into the PIO FIFO.
- **Idle behaviour**: when the host stops streaming, silence is transmitted so
  the receiver stays locked.

## Toolchain

- Raspberry Pi Pico SDK 2.x (RP2350 support, bundled TinyUSB)
- ARM GCC, CMake, Ninja, picotool: all installed by the
  [Raspberry Pi Pico VS Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
- Raspberry Pi Debug Probe (SWD + UART console) for debugging, since the
  Pico's own USB port is occupied by the audio device

## Building

```
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2
cmake --build build
```

Flash with picotool or copy `build/usb2spdif.uf2` to the BOOTSEL drive.

## Status

Project skeleton. See `CLAUDE.md` for the development plan and conventions.

## License

MIT. See `LICENSE`. Copy it, modify it, build on it, sell it, no permission
needed. The firmware links against the Raspberry Pi Pico SDK (BSD-3-Clause)
and TinyUSB (MIT), whose notices apply to those components.
