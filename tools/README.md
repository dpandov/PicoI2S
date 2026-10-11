# Host tools

Scripts that test, monitor and diagnose a PicoI2S device from the PC,
and design and plot its dual-line filters. Run them from the repository
root.

| Tool | Purpose | Host |
|---|---|---|
| [`selftest.py`](#selftestpy) | Bit-exact check of the S/PDIF and I2S outputs (self-test firmware) | Windows, Linux |
| [`stream_test.py`](#stream_testpy) | Plays a tone at every format and checks the device's view of the stream | Windows |
| [`monitor.py`](#monitorpy) | Live event log: sessions, formats, silence, counters | any (libusb) |
| [`diag.py`](#diagpy) | Reads the diagnostic string: crash record and live status | any (libusb) |
| [`i2s_filter_plot.py`](#i2s_filter_plotpy) | Plots the dual-line filters: frequency, impulse, step response | any |
| [`i2s_filter_design.py`](#i2s_filter_designpy) | Designs the filters and writes their coefficient header | any |
| [`i2s_deemph_design.py`](#i2s_deemph_designpy) | Designs the 50/15 us de-emphasis and writes its coefficient header | any |
| `wasapi_play.c` | Native WASAPI exclusive-mode pattern player, built by `selftest.py` | Windows |

## Setup

```
python -m pip install -r tools/requirements.txt
```

`python -m pip` avoids a stray `pip` launcher that belongs to another
Python installation. What each tool needs:

- `diag.py`, `monitor.py`: `pyusb`, `libusb-package`.
- `selftest.py`: those plus `ziglang` (compiler for `wasapi_play.c`) on
  Windows; `numpy` and `aplay` / `amixer` (alsa-utils) on Linux.
- `stream_test.py`: those plus `sounddevice`, `numpy`.
- `i2s_filter_plot.py`: `numpy`, `matplotlib`.
- `i2s_filter_design.py`, `i2s_deemph_design.py`: `numpy`, `scipy` (not in
  the requirements file, only needed to change the filters).

The USB tools read hidden string descriptors through the firmware's picotool
reset interface, which Windows binds to WinUSB automatically (Windows 8.1
or later); no audio driver is involved. They find the device in both modes
(UAC2 PID 0x0001, UAC1 PID 0x0002) and detect the mode from the
diagnostic string. Run the Windows test tools in a local session: over
Remote Desktop, Windows redirects audio and the device's playback endpoint
is not available.

On Linux, libusb needs access to the device; a udev rule gives it to every
user (then replug the device):

```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="000[12]", MODE="0666"' |
    sudo tee /etc/udev/rules.d/60-picoi2s.rules
sudo udevadm control --reload-rules
```

The Windows test tools play in WASAPI exclusive mode, in which Windows still
applies the device's own volume control (Feature Unit). They first set the
device's Windows volume to 100 % and unmute it
(`wasapi_play --set-volume-max`; a failure is only a warning), and fail
while the device reports an attenuation or mute (diagnostic token `v<dB>`
other than `v0`, or `vM`).

## selftest.py

Bit-exact check of the S/PDIF and I2S / dual-line outputs, with no
receiver, DAC, probe or UART. Needs firmware built with
`-DPICOI2S_SELFTEST=ON` (see Self-test in the main README): the device
decodes its own output pins on core 1 and serves the results as USB strings
0x21 (S/PDIF) and 0x22 (I2S). The script plays a test pattern and reads
the reports. Windows: `wasapi_play.c` (built on first use into
`tools/build/`; PortAudio's exclusive mode is not bit-exact). Linux:
`aplay` on the device's ALSA `hw` device (found by name in
`/proc/asound/cards`), fed by the script: `hw` has no format conversion
and no software volume, so it is bit-exact. Nothing else may hold the card
(stop PipeWire / PulseAudio for the test). The script first sets the
card's `PCM` control to 0 dB and unmutes it with `amixer`, since ALSA sets
a new card to -20 dB.

```
python tools/selftest.py                        sweep: 5 rates x 16/24 bit, 3-s stop
python tools/selftest.py --only 44100/16        one format
python tools/selftest.py --soak 60 --rate 96000 --bits 24
                                                one format for 60 minutes
python tools/selftest.py --status               print the three reports once
python tools/selftest.py --i2s-format dual20    require this I2S format (jumpers)
python tools/selftest.py --i2s-filter slow      require this dual-line filter (jumpers)
```

| Option | Meaning |
|---|---|
| `--only RATE/BITS` | sweep one format, e.g. `44100/16` |
| `--no-stop` | skip the 3-s stop test |
| `--soak MINUTES` | play one format continuously and log every glitch |
| `--rate HZ`, `--bits 16\|24` | soak format (default 48000 / 24; 16 in UAC1 mode) |
| `--interval S` | soak read interval (default 5 s) |
| `--device NAME` | playback device name part (Windows endpoint, Linux card; default: the device in either mode) |
| `--i2s-format FMT` | expected I2S format: i2s, lj, rj16 (rj24 with the `PICOI2S_RJ24` build), tda1541a, dual16, dual18, dual20, dual24 (default: what the device reports) |
| `--i2s-filter NAME` | expected dual-line filter: sharp, slow, minphase, sinc, nos (default: what the device reports) |
| `--status` | print the diagnostic, S/PDIF and I2S reports once and exit |
| `-v`, `--verbose` | print every read |

**Sweep.** 48k/16, 48k/24, 44.1k/24, 96k/16, 88.2k/16, 88.2k/24, 32k/24,
32k/16, 44.1k/16, 96k/24: 16 <-> 24 bit at the same rate and clock family
switches in both directions. In UAC1 mode the device offers 16 bit up to
48 kHz only, so the sweep is 48k/16, 44.1k/16, 32k/16; there the output
clock follows the host, the measured rate is the host's clock against the
Pico crystal, and the rate trim and servo lock are reported (a trim at the
servo limit, +-1500 ppm, fails). Each format plays for 7 s and is read at
2.5 ... 6.5 s.

**S/PDIF PASS** means every read showed the stream and the output at the
requested format; a clean window of pattern frames only (`ok`: no silent
frames, no cell, parity, order, V/U or pattern errors); the right channel
status (rate and word length codes); a measured frame rate within
+-2000 ppm; fresh results; no reboot. Also, from the first to the last read,
no bad, gap or skipped windows were added and the underrun, overrun and
late-interrupt counters (`ur`, `ov`, `lt`) did not increase.

**I2S PASS** (a column of its own) means every read showed the I2S window
`ok` at the stream format, in the expected format and filter, with no
sample or framing errors, and no bad, gap, skipped or late (`L`) counts
were added. The filter must run at its oversampling for the rate: sharp,
slow and minphase 8x below 50 kHz, 4x above. How the samples are checked:
- the direct formats (i2s, lj, rj16 or rj24) and dual-line with nos: against
  the pattern, at the format's own width when it is narrower than the
  stream (rj16, tda1541a, or dual16 .. dual20 at 24 bit);
- the filtered dual-line formats (sharp, slow, minphase, and sinc at the
  sample rate): bit-exact against the window's own S/PDIF samples run
  through the same filter on the device, so no host-side model is
  involved.

The I2S column shows the filter token (`sharp8`, `slow4`, `sinc`, `nos`,
`-` for the direct formats). The self-test frames the capture with the
transmitter's own slot layout, so it cannot catch a misplaced slot; that
is covered by the PIO simulation, not here.

**Stop test.** After the sweep the S/PDIF output must send silence 2 s after
the stream closed and be stopped (line idle low) 4.5 s after. The I2S
output must be silent, then stopped with all pins low. With a
`PICOI2S_KEEP_CLOCKS` build (configuration string `clocks=keep`) S/PDIF
stops the same way, but the I2S output must still send silence at the
last rate after 4.5 s.

**Soak.** One format plays continuously and is read every `--interval`
seconds. Every failed check or counter increase is logged with a time
stamp, I2S failures included. It passes only with zero events. Bad windows
are counted on the device, so a glitch between two reads is still caught.

**Test-tone firmware** (`-DPICOI2S_TEST_TONE=ON` as well): the output is
the tone, not the pattern, and the report shows `s-`. The script then checks
the tone's framing, channel status and rate, plus the I2S framing. A
filtered dual-line output still gets its bit-exact check against the
filtered S/PDIF samples (its `s` stays numeric and must be 0).

### Report strings

S/PDIF report (string 0x21; full definition in
`src/selftest/selftest_report.h`):

```
#<seq> <state> tx<rate>/<bits> f<frames> z<silent> e<cell> p<parity>
    o<order> v<V/U> s<pattern|-> c<match><cs3><cs4> fs<Hz>
    B<bad> G<gaps> K<skipped> [@<frame>:<prev L>><L>/<R>]

e.g. #812 ok tx48000/24 f577 z0 e0 p0 o0 v0 s0 c1020B fs47991 B2 G0 K1
```

| Field | Meaning |
|---|---|
| state | `ok`, `sil` (silent frames), `idle` (output stopped, line static), `wait`; failures in capitals: `ERR`, `NOSYNC`, `NOCARR`, `CARRIER`, `CAPTMO`, `NOPIO` |
| `c` | `1` channel status matches, `0` differs, `-` no complete block, `x` C bit differs between subframes; then cs3, the rate code (00 44.1k, 02 48k, 03 32k, 08 88.2k, 0A 96k), and cs4, the length (02 16, 0B 24) |
| `B G K` | cumulative windows: bad, with gaps (dropouts), skipped (format or clock change, volume ramp or source-switch fade during the capture) |

I2S report (string 0x22, same sequence number as the S/PDIF window):

```
#<seq> <state> <fmt> <flt> tx<rate>/<bits> f<frames> z<silent> s<errors|->
    e<framing> L<late> B<bad> G<gaps> K<skipped>
    [@<frame>:<prev or expected L>><L>/<R>]

e.g. #812 ok i2s - tx48000/24 f320 z0 s0 e0 L0 B0 G0 K1
     #812 ok dual16 sharp8 tx48000/24 f1280 z0 s0 e0 L0 B0 G0 K1
```

| Field | Meaning |
|---|---|
| fmt | jumper format: `i2s`, `lj`, `rj16` (`rj24` with the `PICOI2S_RJ24` build), `tda1541a`, `dual16`, `dual18`, `dual20`, `dual24` |
| flt | dual-line filter (jumpers): `sharp`, `slow` or `minphase` with the oversampling of the running output appended (`sharp8`, `slow4`, ...; the bare name while stopped), `sinc`, `nos`; `-` for the direct formats |
| state | `ok`, `sil`, `idle` (stopped, pins low), `wait`; failures: `ERR`, `NOSYNC` (LRCK static / no LATCH), `NOCLK` (no BCLK edges), `NOTIM` (no timing for the format at this rate), `OFF` (S/PDIF runs, I2S does not), `PINHI` (stopped, a pin high: `lvl<hex>` = DATA, DR, BCLK, LRCK in bits 0..3), `NOPIO` |
| `s` | pattern violations (direct, nos) or frames differing from the filtered S/PDIF reference (sharp, slow, minphase, sinc); `-` when no sample check ran |
| `L` | cumulative blocks that missed the I2S output (core 1 late) |

## stream_test.py

Plays a 1 kHz tone at every supported rate and bit depth in WASAPI
exclusive mode (through `sounddevice`) and checks the device's own view of
the stream in the diagnostic string. Works with the normal firmware; no
bit-exact check (use `selftest.py` for that).

```
python tools/stream_test.py
```

A format passes when, during playback, the device reports streaming at the
requested rate and resolution, S/PDIF output at the same rate, audio
playing, LED on, and no underruns or overruns since the stream started.
The longest main-loop iteration, late S/PDIF interrupts and the lowest ring
level are reported. The sweep alternates rate families and bit depths, so
it covers clock switches and word-length changes on a running output.
After the last format the output must keep sending silence for about 3 s
and then stop. In UAC1 mode only 32 / 44.1 / 48 kHz at 16 bit are played,
and the rate trim and servo lock are reported per format.

## monitor.py

Watches a running device (normal or self-test firmware) and prints one
timestamped line per event. It polls the diagnostic string and the session
string (0x23) a few times per second.

```
python tools/monitor.py                         events only, until Ctrl+C
python tools/monitor.py --status 10 --log session.log
python tools/monitor.py --silence 2 --interval 0.5
```

```
16:54:45 device connected: UAC2, non-Apple host, boot 3
16:54:45   config: board Waveshare RP2350-One, firmware 1.03, normal build
16:54:45   config: USB UAC2 (mode jumper GP5 open)
16:54:45   config: output format TDA1541A simultaneous mode 16-bit, DEM clock on (jumpers GP8 GP7 GP6 = 011)
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

Events:
- device connected, disconnected or rebooted (with the crash record of the
  previous boot), each followed by the configuration: board, firmware
  version, build (normal, self-test, test tone), USB mode, output format,
  filter (and whether the format uses it), and the state of every jumper
  (firmware without the configuration string: "not reported");
- stream opened, restarted or closed, with the format, the PCM data rate
  and the volume;
- playing, stalled, underrun;
- S/PDIF output on and off;
- digital silence (all-zero samples, after the volume, for `--silence`
  seconds) and the return of signal;
- volume changes and clock lock (UAC1);
- overrun and late-interrupt counts;
- late I2S blocks (the session string's `os` token: blocks that missed the
  I2S output, normally from the filter path on core 1; 0 in normal
  operation);
- S/PDIF input: lock and loss of lock per input (coax, Toslink), the
  input the receiver selects with its rate and offset, playing and
  stopped playing (lost lock, discontinuity or no samples), USB taking the
  output back or the input taking it after 3 s of USB idle, pre-emphasis
  on and off, non-PCM data (played as silence), and increases of the
  receiver's parity, framing and overflow counters. Firmware without the
  input tokens reports none of these.

```
17:02:11 coax input locked and selected: 44100 Hz, +12 ppm
17:02:14 S/PDIF output on: 44100 Hz
17:02:14 playing: coax input, 44100 Hz, +12 ppm, 16-bit
17:02:40 input pre-emphasis on (50/15 us)
17:03:05 stream opened: 48000 Hz / 16-bit (1.536 Mbit/s), volume 0 dB
17:03:07 playing
17:03:07 S/PDIF output switched to 48000 Hz
17:03:07 USB took the output back from the coax input after 53.0 s
```

| Option | Meaning |
|---|---|
| `--interval S` | poll period (default 0.25 s); events shorter than this are not seen |
| `--silence S` | all-zero samples for this long count as digital silence (default 0.5 s) |
| `--status S` | status line every S seconds while a stream is open: format, data rate, the sample rate measured from the device's frame counter against the PC clock (over 30 s, to about 100 ppm; the USB read timing limits it), FIFO level, ring minimum, main-loop time, trim, current silence. While the S/PDIF input plays: the input, its rate and offset, word length, emphasis, current silence |
| `--log FILE` | append every line to a file |

Ctrl+C prints a summary (stream starts, time streaming).

## diag.py

Reads the diagnostic string (0x20): the previous boot's crash record and
the live stream status, after one line with the configuration string
(0x24, see below). This works without a debug probe or UART.

```
python tools/diag.py            one reading
python tools/diag.py 10         ten readings, one per second
```

```
UAC2:  b3 none | So 48000/24 ff4712 ur0 run1 led1 tx48000 ov0 lt0 rmin144 fmax5240 lp212 v0
UAC1:  b1 none | S1 44100/16 ff4935 ur0 run1 led1 tx44100 ov0 lt0 rmin144 fmax5400 lp212 tr361L v12
```

| Token | Meaning |
|---|---|
| `b<N>` | boot count since power-up or the last flash |
| `none` / `panic` / `fault` / `hang` / `wdog` | how the previous boot ended |
| `m<N>` | (after a crash) last breadcrumb, `DIAG_MARK_*` in `src/diag/diag.h` |
| `pc` / `lr` | (after a crash) code address of the panic message, fault or hang |
| `\|` | everything after the last `\| ` is the application status |
| `D` / `I` / `S` | link state: detached, idle, streaming |
| `a` / `o` / `1` | UAC2 mode: host detected as Apple (3-byte 10.14 feedback) or other; `1`: UAC1 mode |
| rate/bits | current stream format |
| `ff<N>` | receive FIFO level in bytes (depth 9312, held at 6 packets) |
| `ur<N>` | underruns: output ring ran dry while playing |
| `run0/1` | USB stream samples going out on S/PDIF (the S/PDIF input playing shows as `src` in the session string) |
| `led0/1` | status LED state (any colour) |
| `tx<Hz>` | S/PDIF output sample rate, 0 when stopped |
| `ov<N>` | overruns: USB packets that overwrote unread FIFO data |
| `lt<N>` | S/PDIF interrupts too late to refill a buffer |
| `rmin<N>` | lowest output ring level while playing, in frames (target 192) |
| `fmax<N>` | highest receive FIFO level in bytes |
| `lp<us>` | longest main-loop iteration in microseconds |
| `tr<ppm>L/U` | While the output follows an external clock (UAC1 mode, or the S/PDIF input playing): rate trim in ppm, +186 (32 / 48 kHz) or +344 (44.1 kHz) to cancel the PLL error, plus the source clock's offset against the Pico crystal. `L` locked, `U` acquiring or not playing |
| `v<dB>` / `vM` | host volume: attenuation 0 to 64 dB, or muted (or below -64 dB). `v0` is bit-perfect; below 0 dB the output is scaled and flagged as 24-bit |

`ur`, `ov`, `lt` and `rmin` count from the start of the current or last
stream (open or format change); `fmax` and `lp` from when it began
playing, so the start-up is excluded. Counters stop at 999999 (9999 in
UAC1 mode).

The session string (0x23, read by `monitor.py` and printed by `diag.py`
after each reading) carries counters since boot and the S/PDIF input
state, at most 123 characters:

```
n<streams> fr<frames> sig<ms|-> up<ms> os<late> src<u|c|t|-> in<c|t|->/<c|-><t|->
    ir<rate><+|-><ppm>|ir- cs<bits>[e][n]|cs- rx<locks>/<unlocks>/<parity>/<framing>/<overflows>

e.g. n3 fr91234567 sig12 up734002 os0 srcc inc/ct ir44100+12 cs24e rx3/1/0/2/0
```

| Token | Meaning |
|---|---|
| `n<N>` | USB stream starts (open, or format change while open) |
| `fr<N>` | frames moved to the output, from either source (wraps at 2^32) |
| `sig<ms>` | milliseconds since the last non-zero sample went out (`-` if none yet) |
| `up<ms>` | uptime |
| `os<N>` | I2S blocks that missed their DMA buffer |
| `src` | source heard: `u` USB, `c` coax input, `t` Toslink input, `-` none (pre-roll, underrun, stall, output off) |
| `in` | input the receiver selects (`c`, `t`, `-` when neither is locked), then each input's own lock (`c` / `t`, or `-`) |
| `ir` | nominal rate of the selected input and its measured offset in ppm; `ir-` without one |
| `cs` | its channel status: word length, `e` for 50/15 us pre-emphasis, `n` for non-PCM data (played as silence); `cs-` until a whole block has arrived |
| `rx` | receiver counters: lock acquisitions, lock losses, parity errors, preamble / framing errors, FIFO overflows |

Counters from `os` on stop at 99999, the offset at +/-9999 ppm. Older
firmware ends after `up` (or `os`); the tools treat the missing tokens as
not reported.

The configuration string (0x24, printed by `diag.py` before its readings
and by `monitor.py` on connect) is fixed at boot:

```
board=<rp2350-one|pico2> fw=<bcdDevice> build=<normal|selftest|tone|tone+selftest>
    mode=<uac2|uac1> format=<i2s|lj|rj16|rj24|tda1541a|dual16|dual18|dual20|dual24>
    filter=<nos|sharp|slow|sinc|minphase> jumpers=<GP5>/<GP8 GP7 GP6>/<GP2 GP3 GP4>
    clocks=<stop|keep>

e.g. board=rp2350-one fw=0104 build=normal mode=uac2 format=i2s filter=nos jumpers=0/000/000 clocks=stop
```

`filter` is what the filter jumpers select (reserved codes: sharp), also
for I2S / LJ / RJ, which never use it; in `jumpers` 1 is grounded.

## i2s_filter_plot.py

Plots the dual-line filters from the coefficients the firmware runs
(`src/i2s/i2s_filter_coef.h`, parsed, so the plots follow the committed
header). There are four panels: the frequency response over the full
range, the same zoomed around Fs/2 with a passband inset, the impulse
response and the step response.

```
python tools/i2s_filter_plot.py                         all filters at 44.1 kHz
python tools/i2s_filter_plot.py --rate 96000 --filter sharp,minphase
python tools/i2s_filter_plot.py --no-hold --save filters.png
```

| Option | Meaning |
|---|---|
| `--rate HZ` | sample rate (default 44100); selects 8x below 50 kHz, 4x above, as the firmware does |
| `--filter LIST` | comma-separated: sharp, slow, minphase, sinc, nos (default all) |
| `--no-hold` | digital response only, without the DAC's zero-order hold |
| `--fmax HZ` | range of the full plot (default 4 x rate) |
| `--save PNG` | write the figure to a file instead of opening a window |

By default the DAC's zero-order hold at the output word rate is included,
so the curves are what leaves the DAC before its analogue filter; that
makes NOS and the sinc compensation comparable with the oversampling
filters. Above Fs/2 the frequency plots show what an input tone at the
alias frequency leaves as images. Slow and minimum phase have the same
magnitude, so minimum phase is dashed. The time axes are in input samples
from the impulse or step, so the latency (group delay, in the legend)
shows. Responses are scaled to unity gain at DC; sinc is at -4 dB by
design. The console gets a summary per filter: taps, group delay, and the
level at 20 kHz, Fs/2 and 0.55 Fs.

The window uses Tk. A `TCL_LIBRARY` environment variable left by another
program (CSR BlueSuite sets one machine-wide) breaks Tk with "Can't find a
usable init.tcl"; the tool ignores such a variable for its own process.
`--save` needs no window at all.

## i2s_filter_design.py

Designs the interpolation filters, checks them and writes
`src/i2s/i2s_filter_coef.h`. The header is committed, and a rerun rewrites
it identically. Run it only to change the filters.

```
python -m pip install numpy scipy
python tools/i2s_filter_design.py
```

It prints for each cascade the passband ripple, the level at 0.4535 Fs,
the worst image and the gain bound. It stops without writing the header
when a spec is not met. The specs are images at least 100 dB down (the slow
filters' first image excepted) and a gain bound below 8, which the 1/16
input scaling in `src/i2s/i2s_filter.h` relies on. The filters and stage
layouts are described at the top of the script and in the main README
(Filters).

## i2s_deemph_design.py

Designs the de-emphasis for sources flagged with 50/15 us pre-emphasis,
checks it and writes `src/i2s/i2s_deemph_coef.h`. The header is committed,
and a rerun rewrites it identically. Run it only to change the design.

```
python -m pip install numpy scipy
python tools/i2s_deemph_design.py
```

One biquad per sample rate (32, 44.1, 48, 88.2, 96 kHz), its magnitude
fitted minimax in dB to the analogue curve (1 + s 15 us) / (1 + s 50 us)
over 20 Hz .. 20 kHz (0.45 Fs at 32 kHz), with the gain held at or below
0 dB. The coefficients are Q30 for the direct form I in
`src/i2s/i2s_deemph.h`. The tool prints per rate the magnitude error, the
phase deviation (raw, and after removing the constant lead of a few us,
which is a pure time shift), the peak and DC gain, the poles, and the
output margin. It stops without writing the header when a check fails on
the quantised coefficients: magnitude error above 0.01 dB, gain above
0 dB, a pole outside the unit circle or not real, a negative impulse
response, feedback coefficients summing to 1 or more (needed for the
state to decay to exactly zero), a possible accumulator overflow, or a DC
margin too small for the rounding error (24-bit input must never reach the
output clamp).
