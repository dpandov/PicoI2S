# CLAUDE.md

Guidance for Claude Code in this repository. Read `README.md` first (what
the device does, wiring, jumpers, building). `DESIGN.md` explains how it
works inside, why, and what was measured; `tools/README.md` is the manual
of the host tools.

## Project

PicoI2S: USB audio interface firmware for the RP2350, the Waveshare
RP2350-One (default) or the Raspberry Pi Pico 2. It drives an I2S or
dual-line multibit DAC through on-chip filters, sends the same samples as
S/PDIF, and plays an external S/PDIF input when USB is idle. USB names:
"Pico USB to I2S" (UAC2), "Pico I2S" (UAC1); the host tools match both.

Requirements that drive every change:

- USB Audio Class 2.0, asynchronous with a feedback endpoint: the Pico is
  the clock master. Never resample (an ASRC was prototyped for the
  S/PDIF input and held back; using it needs the owner's explicit
  exception).
- UAC1 mode (GP5 grounded at boot) is adaptive like the PCM2704: the
  output clock follows the host through a fractional trim of the PIO
  divider. Samples stay bit-exact there too.
- Rates 32 / 44.1 / 48 / 88.2 / 96 kHz, 16 and 24 bit, stereo (UAC1:
  32 / 44.1 / 48 kHz, 16 bit).
- Primary focus: 44.1 and 48 kHz. Every other rate is a bonus: it must
  keep working, but it does not drive design decisions. Base research,
  trade-offs and proposals (filters, clocks, cycle budgets, latency) on
  44.1 / 48 kHz first, and do not give up quality there to fit 32 / 88.2 /
  96 kHz; a bonus rate may get a simpler or cheaper variant instead.
- When audio stops (stream closed or no samples arriving), send S/PDIF
  silence for 3 s, then stop every output (lines low, no carrier); restart
  on the next stream. Digital silence inside an open stream is still
  audio. The `PICOI2S_KEEP_CLOCKS` build keeps the DAC clocks running
  with silence instead; S/PDIF still stops.
- clk_sys is switched per rate so every PIO divider is an integer; the
  UART stays on the 48 MHz USB PLL.
- LED: on while audio plays, two short flashes on enumeration, no other
  blink patterns; it is reserved for that (the RP2350-One's colour shows
  the source, the Pico 2 adds LOCK and EMPH LEDs for the input).
- All jumpers open = UAC2, I2S, no filter (NOS). Keep it that way.

Pins (`src/board_config.h`; RP2350-One, Pico 2 in brackets): S/PDIF out
GP12 [GP28]; I2S DATA / DL GP14, DR GP15, BCLK GP26, LRCK / LATCH GP27,
MCLK / DEM GP28 [GP16..GP20]; S/PDIF in coax GP10, Toslink GP11 [GP26,
GP27]; mode jumper GP5; format jumpers GP6..GP8; filter jumpers GP4..GP2;
UART0 console GP0 / GP1; LED WS2812 GP16 [GP25, LOCK GP22, EMPH GP21].
Jumpers have pull-ups, are read once at boot, grounded = 1.

## Toolchain

- Pico SDK 2.x, ARM GCC, CMake, Ninja, picotool, installed by the
  Raspberry Pi Pico VS Code extension into `~/.pico-sdk`; versions pinned
  at the top of `CMakeLists.txt`. ARM cores, C.
- TinyUSB 0.21.0 is the submodule `lib/tinyusb` (release tag), passed as
  `PICO_TINYUSB_PATH`; the SDK's bundled copy is not used. After cloning:
  `git submodule update --init`.
- Version: `project(PicoI2S VERSION x.y)` in `CMakeLists.txt`, shown as
  x.0y and also the USB bcdDevice: bump it on every release and every
  descriptor change (Windows caches descriptors per bcdDevice). The build
  number (commit count), commit and build time come from
  `cmake/version.cmake` at every build (version string 0x25).
- `PICO_BOARD=pico2` for both boards; the wiring is `PICOI2S_BOARD`
  (`rp2350-one`, default, or `pico2`). Build options: `PICOI2S_SELFTEST`,
  `PICOI2S_TEST_TONE`, `PICOI2S_RJ24`, `PICOI2S_KEEP_CLOCKS`.
- Build: `cmake -S . -B build -G Ninja -DPICO_BOARD=pico2 && cmake --build build`
  (`build/` belongs to the VS Code extension; Claude's builds go in
  `scratch/`). Outside the VS Code terminal, in Git Bash:
  `P=~/.pico-sdk; export PICO_SDK_PATH=$P/sdk/<ver> PICO_TOOLCHAIN_PATH=$P/toolchain/<ver> PATH=$P/toolchain/<ver>/bin:$P/picotool/<ver>/picotool:$P/cmake/<ver>/bin:$P/ninja/<ver>:$PATH`
- Flash without BOOTSEL (the firmware has the picotool reset interface):
  `picotool load -x <uf2> -f --vid 0x1209 --pid 0x0001` (UAC1: 0x0002).
  BOOTSEL is needed only after a hang with the watchdog off or after
  other code.
- No debug probe or UART adapter is connected: use the diagnostic string
  (`tools/diag.py`), which reports panics, hard faults and hangs from the
  previous boot plus live status. The console is UART0 at 921600 baud.

## Testing

- The self-test build (`-DPICOI2S_SELFTEST=ON`) decodes the S/PDIF and
  I2S outputs on core 1; `tools/selftest.py` plays a pattern through a
  native WASAPI exclusive player (`tools/wasapi_play.c`; PortAudio is not
  bit-exact), checks both outputs bit-exact, the 3-s stop, and soaks
  (`--soak`). Run it after any change to the audio path. A source on the
  S/PDIF input fails the 3-s stop check (the input takes the output):
  unplug it or pass `--no-stop`.
- The tools set the endpoint volume to maximum first: Windows applies the
  device's own volume in exclusive mode. On Linux, ALSA starts a new card
  at -20 dB; bit-perfect playback needs `PCM` at 0 dB.
- Proof that a host follows the feedback: a scratch build that consumes a
  few thousand ppm fast or slow must keep the FIFO stable without
  underruns. Never commit such code.
- Scratch-only harnesses (they may be gone; rewrite if needed):
  `scratch/i2s-sim` (cycle-accurate PIO simulation of every generated
  program against datasheet models of the DACs), `scratch/selftest-os-test`
  (host tests of the self-test decoders), `scratch/fmt-remap/sweep.sh`
  (sweeps with a forced format, filter or UAC mode), `scratch/e2e-input`
  (test transmitter on the clock pin for the input), `scratch/load`
  (core-0 load meter), `scratch/mclk-board` (MCLK meter), `scratch/loop`
  (optical loop tests).

## Scratch folder rule

All experiments, test programs, throwaway builds, captured data and
generated analysis go in `scratch/` at the repository root. This folder and
`temp/` are gitignored and must stay that way.

- Any test, probe or spike Claude writes is created and run from
  `scratch/`, never from the source tree.
- `scratch/` may be deleted at any time; nothing in it is part of the
  project. Something that proves useful is rewritten properly into the
  source tree in a separate step, not moved.
- Do not add build directories or scratch content to git. Check
  `git status` before committing.

## Conventions

- One folder per subsystem: `src/main.c` (init and main loop only),
  `src/board_config.h` (pins), `src/usb/` (descriptors and every `tud_*`
  callback), `src/app/` (pipeline, clock servo, LED, monitor strings, version),
  `src/diag/`, `src/spdif/` (transmitter, trim pacer, receiver),
  `src/i2s/` (I2S / dual-line transmitter, filters, de-emphasis),
  `src/clock/` (PLL per rate), `src/selftest/`, `tools/` (host scripts),
  `cmake/` (build scripts).
- Includes are relative to `src/` (`#include "usb/usb_audio.h"`).
- Do not mix TinyUSB callbacks with PIO / DMA code.
- Prefer integer arithmetic in the audio path; no floating point in
  interrupt handlers.
- Comments: concise and to the point, only where the code does not speak
  for itself or something is important or ambiguous (e.g. the reason for
  a PLL value). No planning, stage or history notes in code; those
  belong in commits and `DESIGN.md`.
- Keep the docs current with the code: `README.md` for users,
  `DESIGN.md` for design, measurements and the verification record,
  `tools/README.md` whenever a tool changes, this file for rules and
  status, and an entry under "Unreleased" in `CHANGELOG.md` for every
  user-visible change. `CONTRIBUTING.md` restates the rules for outside
  contributors; keep it in step.
- Commit messages: short imperative subject, body explaining why.

## Status

| Area | State |
|---|---|
| UAC2 USB audio | done; Windows 11, Linux, iPad / iPhone, Android |
| UAC1 mode | done; Windows 11, Linux, iPad / iPhone |
| Volume (Feature Unit, ramp) | done |
| Latency (6 ms FIFO set point) | done |
| Robustness, self-test, long soaks | done (record in `DESIGN.md`) |
| Host monitor (`tools/monitor.py`) | done |
| I2S / dual-line output, five filters, MCLK, DEM | done; AD1865 in dual18 played on real hardware |
| Format map with the PCM56 mode, LATCH timing | done; board sweeps of every format, AD1865 listening test, scope check |
| Trim pacer, I2S alignment loop, OUT re-arm | done |
| Source-switch fades, keep-clocks build option | done; input harness, self-test in UAC1 and UAC2, 5.5-h soak |
| S/PDIF input, arbitration, de-emphasis | done; Toslink with a real source and in an optical loop |

Pending:

- Hosts: a TV (UAC1), macOS; Android after the rename (new strings,
  terminal type 0x0602).
- I2S: bring-up on an I2S DAC board (PCM5102 / ES9023), a TDA1541A
  board (DEM clock), a PCM56 pair, a dual-line soak in both modes.
- S/PDIF input: a real coax receiver (margins at 96 kHz), the TDA1541A
  format with the input, the Pico 2 LOCK / EMPH LEDs, and as public
  features a self-test loopback (S/PDIF out to an input pin) and a
  self-test transmitter with the emphasis flag.
- Known limitations are listed in `DESIGN.md`.

## Known TinyUSB issues (0.21.0)

- The ISO receive path runs in the USB interrupt (`xfer_isr`): FIFO write,
  re-arm, FIFO_COUNT feedback and `tud_audio_rx_done_isr()`. State shared
  with that callback needs interrupt-safe access from the main loop.
- UAC2 feedback is always 16.16 in 4 bytes; for Apple hosts `usb_audio.c`
  rewrites the transfer to 10.14 in 3 bytes (`--wrap=usbd_edpt_xfer` in
  `CMakeLists.txt`). Keep that workaround.
- UAC1: a data endpoint without a sync type is taken for a feedback
  endpoint; the UAC1 data endpoint is declared adaptive (0x09).
- A zero-length ISO OUT packet leaves the endpoint unarmed
  (`audiod_rx_xfer_isr()` returns before the re-arm). `check_rx_halt()`
  in `usb_audio.c` re-arms an idle endpoint of an open stream from the
  main loop with the driver's own buffer (recorded by the
  `usbd_edpt_xfer` wrapper), in one critical section, at most once per ms,
  giving up after three failures until the next stream.
- Feedback parameters are read only at SET_INTERFACE, not on a UAC2 rate
  change while streaming (UAC1 re-reads them on the endpoint SET_CUR).
- The feedback clamp is whole samples per ms: the 44.1 kHz family may run
  at most about 2000 ppm slow. The -63 ppm PLL error is inside that.
- `tud_umount_cb()` is called on unplug but not on a bus reset;
  `usb_audio.c` polls `tud_mounted()`.
- Since 0.19 the audio driver aborts a pending transfer when it activates
  an ISO endpoint, so reopening a stream no longer panics ("ep 01 was
  already available"; a regression would read "buf_ctrl ... already
  available" in the diagnostic string). Not seen since in the open /
  close and format-switch loops on Windows and Linux.
- On RP2350 the USB interrupt handler runs from flash; the S/PDIF DMA
  interrupt has higher priority and stays in RAM.

## Host compatibility

- Full Speed feedback: Windows needs a 4-byte endpoint with 16.16 (Code 10
  with 3 bytes), Apple 3 bytes with 10.14. `usb_descriptors.c` detects
  Apple hosts (string request before the configuration request after the
  device descriptor) and serves a second configuration; the diagnostic
  string shows the guess (`a` / `o`).
- PortAudio's WASAPI exclusive mode (`sounddevice`) replays a stale half
  buffer now and then; a 1 kHz tone hides it, so `tools/stream_test.py`
  still works, but bit-exact tests need the native player.
- Windows hides the USB audio device while the PC is used over Remote
  Desktop.

## Git rules

- Branches: `main` holds releases only, one commit "Release x.yy" each,
  tagged `vx.yy`. A release: commit "Release x.yy" on `develop` (version,
  changelog date), pull request `develop` -> `main` squash-merged on
  GitHub, tag on the new `main` commit, then `git merge -s ours
  origin/main -m "Merge main into develop after release x.yy"` on
  `develop` so the next pull request diffs from the release. Build
  numbers on `main` count releases, on `develop` all commits.
  `develop` integrates; work goes on `feature/<topic>` / `fix/<topic>`
  from `develop`. No CI: release files are built from the tag and
  attached to the GitHub release by hand.

- Do not add "Co-Authored-By" or any other Claude / AI attribution lines to
  commit messages or pull request descriptions in this repository.
- Never run `git push` (or any command that publishes to a remote). Pushing
  is done by the project owner only. Local commits are fine when asked.
