# Contributing

Bug reports, measurements on new DACs and hosts, and pull requests are
welcome. The [README](README.md) explains what the device does and how to
build it; [DESIGN.md](DESIGN.md) explains how it works inside and what has
been measured; [tools/README.md](tools/README.md) documents the host
tools.

## Reporting a problem

Open an issue with the bug report form. The most useful information is
what the device says about itself: run `python tools/diag.py` (one
reading) and paste the output. It contains the firmware version and build,
the configuration (board, USB mode, format, filter, jumpers), the live
state and the crash record of the previous boot. For a problem over time,
a `python tools/monitor.py --status 10 --log monitor.log` log of it helps
most.

## Branches

- `main` holds the releases only: one commit "Release x.yy" per release,
  tagged `vx.yy`, with the complete state of `develop` at that point.
  The development history is on `develop`.
- `develop` is where work is integrated. Base your branch on `develop`
  and open the pull request against it.
- Name branches `feature/<topic>` or `fix/<topic>`, one change each.

## Requirements that drive every change

- The Pico is the clock master (UAC2, asynchronous with feedback); UAC1 and
  the S/PDIF input are followed by trimming the output clock. Samples are
  never resampled and stay bit-exact at 0 dB.
- 44.1 and 48 kHz come first: the other rates must keep working, but
  quality at 44.1 / 48 kHz is not traded for them.
- All jumpers open stays UAC2, I2S, no filter.

## Code

- C for the ARM cores, Pico SDK and TinyUSB at the versions pinned in
  `CMakeLists.txt` and `lib/tinyusb`.
- One folder per subsystem (`src/usb`, `src/app`, `src/spdif`, `src/i2s`,
  `src/clock`, `src/diag`, `src/selftest`); includes relative to `src/`.
  TinyUSB callbacks stay in `src/usb`, apart from the PIO / DMA code.
- Integer arithmetic in the audio path; no floating point in interrupt
  handlers.
- Comments only where the code does not speak for itself (e.g. the reason
  for a PLL value), short, without history; history belongs in the commit
  message and `DESIGN.md`.
- Update the docs with the code: `README.md` for users, `DESIGN.md` for
  design and measurements, `tools/README.md` for the tools, and an entry
  under "Unreleased" in `CHANGELOG.md` for anything a user would notice.
- Commit messages: a short imperative subject, a body that explains why.

## Testing

Changes to the audio path need a board and a Windows PC (or Linux for the
self-test):

1. Build the self-test firmware: `-DPICOI2S_SELFTEST=ON`.
2. Run `python tools/selftest.py`: every format must pass bit-exact on the
   S/PDIF and I2S outputs, and the outputs must stop 3 s after the audio
   (unplug any S/PDIF input source, or pass `--no-stop`).
3. Run it in both USB modes (UAC2, and UAC1 with GP5 grounded), and with
   your DAC format and filter if they differ from the defaults.
4. For timing-sensitive changes, a soak: `python tools/selftest.py --soak
   120 --rate 44100 --bits 16`, and the same at 48 kHz.

Mention in the pull request what you ran and on which board, host and DAC.

## Versions and releases

The version is set in `CMakeLists.txt` (`project(PicoI2S VERSION x.y)`)
and is also the USB device release: it is raised for every release and
every USB descriptor change (Windows caches descriptors per version), the
major number only for changes that break existing setups (USB identity,
pins, jumper meanings). A release goes through a pull request:

1. On `develop`, a commit "Release x.yy" sets the version and dates the
   changelog.
2. A pull request from `develop` to `main` titled "Release x.yy", merged
   with "Squash and merge": one commit on `main`.
3. The tag `vx.yy` on that commit (GitHub release, or
   `git tag -a vx.yy origin/main`), with the firmware files for both
   boards built from it.
4. `main` merged back into `develop`, keeping `develop`'s files, so the
   next pull request shows only the new changes:
   `git merge -s ours origin/main -m "Merge main into develop after release x.yy"`.

A build from `main` counts the release commits as its build number, a
build from `develop` all of its commits.

## License

Contributions are accepted under the project's [MIT license](LICENSE).
