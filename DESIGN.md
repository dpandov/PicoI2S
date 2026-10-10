# Design notes

How the firmware works inside, why it is built that way, and what was
measured. `README.md` is the user's view (wiring, jumpers, building,
tools); `CLAUDE.md` holds the working rules and the current status.

## Clocks

- clk_sys is switched per sample rate so that every PIO divider is an
  integer: the fractional divider stretches single cycles, which would
  jitter the S/PDIF cells. The UART stays on the 48 MHz USB PLL, so
  logging survives a switch. Tables and reasoning: `src/clock/audio_clock.c`.
- Accurate table (UAC2 with the dual-line formats): the most accurate
  in-spec settings that are still fast enough for the filters on core 1:
  102.4 MHz at 32 kHz and 153.6 MHz at 48 kHz (0 ppm), 152.4 MHz at
  44.1 kHz (VCO 1524 / 5 x 2, S/PDIF divider 27) and 101.6 MHz at
  88.2 kHz (5 x 3, divider 9), both -63 ppm, the best below a 1600 MHz VCO;
  147.43 MHz at 96 kHz (-186 ppm; the exact 61.44 MHz is too slow for the
  filters). 152.4 and 153.6 MHz are 1.6 and 2.4 % above the 150 MHz rating.
- Fast table (147.43 / 135.43 MHz, -186 / -344 ppm): UAC1, whose servo
  cancels the error, and the direct I2S formats in UAC2, because the
  256 Fs MCLK needs an S/PDIF divider that is a multiple of 4 (36 / 24 /
  24 / 12 / 12; the accurate 25 / 27 / 25 / 9 are odd). An exhaustive
  search found no in-spec even-divider setting more accurate with a 50 %
  MCLK (44.1 kHz at -63 ppm exists only with a 44 % duty, 48 kHz exact
  only at 61.44 MHz with 40 %).

## S/PDIF transmitter (`src/spdif/spdif_tx.c`)

- Biphase-mark encoding on the CPU (table lookup with line-level
  tracking; the preambles are constants thanks to even parity). The PIO
  program is `out pins, 1` at the cell rate, 128 x Fs.
- Two chained DMA channels over two 96-frame buffers; the completion
  interrupt re-encodes the finished buffer. A frame counter mod 192
  drives the block preamble and channel status.
- Channel status: consumer, PCM, copy permitted, category general, rate
  code and word length from the format, emphasis from the source's flag
  (S/PDIF input only). V = U = 0.
- API: `spdif_tx_init(pin)`, `spdif_tx_start(rate, bits, source, options)`,
  `spdif_tx_stop()`; the source callback delivers 32-bit left-justified
  stereo samples, silence when it has none. Stopped = pin low.
- Test tone (`PICOI2S_TEST_TONE`): 1 kHz at -18 dBFS, 48 kHz / 24-bit,
  integer table of 48 samples, no host needed.

## USB audio pipeline (`src/app/audio_pipeline.c`)

- The main loop moves PCM from the TinyUSB FIFO into a ring of 32-bit
  frames (topped up to 192 frames); the DMA interrupt drains the ring.
  The interrupt never reads the TinyUSB FIFO: the driver clears it before
  calling the close callback.
- One stop rule: the output sends silence and stops 3 s after the last
  stream sample entered the ring, which covers both a closed stream and a
  stall. Digital silence inside an open stream is still audio.
- Start: pre-roll until the FIFO holds its set point plus what the start
  pulls out (ring target and two DMA buffers), so a stream starts with
  steady-state buffering; then the output starts, switching the PLL first
  if needed. Same rate with another word length: the channel status
  changes in place. Another rate: stop, switch, restart.
- Latency: the FIFO set point is 6 packets of the current format
  (`FIFO_SETPOINT_PACKETS`, 6 ms everywhere; it used to be half the 9312-
  byte FIFO, 6 ms at 96/24 but 36 ms at 32/16). Worst case through the
  device 10 / 14 / 18 ms at 96 / 48 / 32 kHz. In UAC2 the threshold goes
  to TinyUSB's FIFO_COUNT feedback (`tud_audio_feedback_params_cb()`, at
  SET_INTERFACE); the gain scales with it, a time constant of about 0.3 s.
  The ring target and the 96-frame blocks stay, as the block is tied to
  the 192-frame channel-status period.
- Robustness counters per stream (diagnostic string): underruns, overruns,
  late DMA interrupts (> 1 buffer: detected, recovered, counted), minimum
  ring level, maximum FIFO level, longest main-loop pass. UART logging is
  non-blocking.

## UAC1 mode

- Selected by GP5 grounded at boot (read before `tusb_init()`, no switch
  at runtime), own PID 0x0002 because Windows caches the driver binding
  per VID / PID. PCM2704-like: device class 0 without IAD, one 16-bit
  stereo alt setting, 32 / 44.1 / 48 kHz discrete, adaptive ISO OUT
  endpoint (0x09; see the TinyUSB notes), no feedback endpoint, rate by
  endpoint SET_CUR, 48 kHz until the host sets one (reset on every
  enumeration), EP 196 bytes. Product and function string "Pico I2S"
  (UAC2: "Pico USB to I2S"; Windows and iOS show the function string,
  Android the product). Output terminal type 0x0602, digital audio
  interface, in both modes (Windows prefixes the endpoint name with it). The reset interface and BOS / MS OS 2.0 stay.
  GET_CUR answered, other endpoint requests STALLed.
- Adaptive clock (`app/clock_servo.c`): the PLL and integer divider stay
  as in UAC2; a control loop on the FIFO level trims the output by
  `spdif_tx_set_trim_ppb()` (see the trim pacer). Started in
  `begin_playing()`, seeded with minus the PLL error on a fresh output,
  or resumed warm (`clock_servo_resume()`: same rate, locked within the
  last 2 s, no re-acquisition swing); held through underruns and the
  silence tail; the pre-roll leaves out the DMA buffers on a running
  output so a restart begins inside the servo's reference window. Lock,
  calm and stage bands are fractions of 24 ms of audio (`BAND_MIN_NS`),
  and a stage drop is decided on the raw window error above a quarter of
  the set point (`drop_ns`), so a 1500 ppm rate step at 32 kHz keeps a
  third of the FIFO. UAC2 never trims.
- Hosts select alt 1, then send the rate: a rate change on an open stream
  is a new stream (stop, switch, restart).
- Diagnostic string: `1` in place of the Apple flag, `tr<ppm><L|U>`
  appended, counters capped at 9999 to fit 95 characters.

## Volume (Feature Unit)

- Why: iOS offers a volume slider only for a device that advertises one.
- Master mute and volume in both classes, -64..0 dB in 1 dB steps (RANGE
  -16384..0, res 256; below -64 dB, -inf included, is silence and CUR
  reads -64 dB). Feature Unit ID 2; UAC1 uses the PCM2704 layout
  (bControlSize 1, master 0x03). TinyUSB 0.21 routes both classes to
  `tud_audio_get/set_req_entity_cb()`; anything but the master channel
  STALLs. 0 dB, unmuted at boot; bcdDevice was bumped for the topology.
- Applied in the main-loop conversion (`pump()`), never in the interrupt:
  0 dB is the unchanged copy (bit-perfect); otherwise a Q31 gain table
  (65 entries, generated offline), one rounding to 24 bits, no
  saturation needed. The S/PDIF word length is 24 while the applied
  attenuation is not 0 (mute included), updated per chunk before the
  chunk is published (`sync_word_bits()`).
- Ramp like the PCM2704: the applied attenuation moves 1 dB per chunk
  (up to 96 frames) towards the target, so jumps, mute and unmute fade; a
  new stream starts at the target. The diag token `v<dB>` / `vM` shows the
  target. Volume applies to every output (it is in the ring).

## Trim pacer (`src/spdif/spdif_pacer.pio`, `spdif_trim.h`)

- In the trimmed modes (UAC1, the S/PDIF input) the trim can only set the
  PIO divider's 8-bit fraction, one FRAC step being 1/(256 x INT) of the
  rate (651 ppm at INT = 6, 163 ppm at INT = 24). Each trimmed state
  machine's CLKDIV therefore switches between the two adjacent FRAC values
  with a fixed 25 us period (`SPDIF_TX_DITHER_US`).
- Hardware does it, no per-period CPU work: a 4-instruction pacer state
  machine per trimmed clock (at clk_sys, on pio0 beside the receiver)
  counts each half's delay and pushes the CLKDIV value; one DMA channel
  moves it into CLKDIV (both edges take the same path, so the latency
  cancels), another feeds the pacer endlessly from a 1 KB ring of 64
  periods x (delay, value). The ring carries the fraction as a first-order
  sigma-delta over its 64 periods (about 0.0015 ppm resolution at INT = 6,
  exact mean rate); `spdif_tx_set_trim_ppb()` rewrites it in place every
  48 ms (a half-updated pair mis-times one half period at most). The
  transmitter owns both pacers; the I2S companion hands over its divider
  and CLKDIV register (`trim_divider()`).
- Phase error: a 40 kHz triangle of f(1-f) x T / (256 x INT) clk_sys
  cycles peak to peak, at most T / (1024 x INT): about 4 ns at INT = 6
  (direct formats at 44.1 / 48 kHz), 8 ns at INT = 3, 1 ns on S/PDIF; zero
  on a whole FRAC step; lines below 20 kHz under 0.02 ns. The hardware
  divider's own single-cycle stretches (about 7 ns) remain. Every edge of
  one state machine moves together; the dual-line LATCH edge carries it as
  sampling-time jitter; MCLK and DEM follow BCLK. UAC2 USB playback has
  none.
- Rejected on measurement: a sigma-delta decided every 25 us in a timer
  interrupt (11 % of core 0, its tone in the audio band) and a
  fixed-period scheme with three interrupts per period (18 %). With the
  pacer the core-0 interrupt load in UAC1 at 44.1 kHz is 8.4 % in total
  (USB and DMA), against 19.8 % with the interrupt dither.

## I2S and dual-line output (`src/i2s/`)

### Structure

- `spdif_tx` stays the timing master and hands every block it fetches to
  a companion (`spdif_tx_companion_t`, registered by `i2s_tx_init()`),
  which formats it into a ring of four DMA buffers. The state machines
  share the transmitter's PIO block, start in one write
  (`pio_enable_sm_mask_in_sync`) and consume one input frame per the same
  number of clk_sys cycles, so the outputs stay sample-aligned.
- Alignment loop (trimmed modes): the two state machines are paced from
  separate rings, so their mean rates could differ by the rings'
  rounding (up to about 20 ppb, an I2S block skipped or repeated per
  day). Every live fill adds how far its target buffer lies ahead of the
  I2S DMA; at each 48 ms servo window `companion_trim_offset_ppb()` takes
  the mean, the first window as reference, the drift (modulo two
  buffers) and returns -5000 ppb per buffer of drift, clamped to
  +-2000 ppb, which `spdif_tx_set_trim_ppb()` adds to the companion's
  pacer only. Time constant 200..600 s, steady lead under a frame, the
  offset quantised to 7..20 ppb. Modelled over 3 h at every rate; on the
  board (UAC1, 44.1 kHz) an injected +1000 ppb mismatch settled to
  -1050 ppb and 40 words of lead in 15 min with no late block, and
  +10000 ppb pinned the clamp and slipped as predicted. Exposed as
  `i2s_tx_trim_offset_ppb()` / `i2s_tx_phase_drift_words()`.
- Stops with S/PDIF after the 3-s silence; clocks low when idle.

### Clock pin (`I2S_CLOCK_PIN`)

A third state machine on the block, at clk_sys (divider 1), with a
program generated per start, slaved to BCLK read back from the pad with
`wait gpio`, so it follows the I2S divider, trimmed or not, without
divider writes of its own (a clock on its own dithered divider wandered
against BCLK on the board).

- Direct formats: MCLK = 256 Fs (`i2s_mclk_program_build()`), two MCLK
  periods of 2 D cycles per BCLK half, restarted on every BCLK edge, so a
  stretched or shortened BCLK half moves the last MCLK low half; MCLK
  rises a fixed 3 clk_sys after each BCLK edge. Needs N a multiple of 8
  and D >= 2 (3 with trim room): N = 8, D = 9 / 6 / 6 / 3 / 3 at 32 ..
  96 kHz. Fast slew, up to 24.6 MHz.
- TDA1541A format: the DEM clock (`i2s_dem_program_build()`), high for k
  BCLK periods and low for k, always at the oversampled word rate (k = 8
  with the 8x / 4x filters, 1 with NOS / sinc at 32 .. 48 kHz, 2 at 88.2 /
  96 kHz; `i2s_dem_half_bclks()`); every edge 4 clk_sys after a BCLK edge,
  50 % duty, constant phase to LATCH. Feeds the TDA1541A's DEM oscillator
  pin in place of the capacitor.
- Every other dual-line format: a plain low GPIO.

### Timing (`i2s_format.h`)

- One input frame is 128 x div clk_sys cycles (div: the S/PDIF divider),
  one output frame C_out = 128 x div / OSR = F slots x N state-machine
  cycles x divider D. `i2s_select_timing()` takes the smallest F at or
  above the format's minimum (64 for I2S / LJ / RJ, the word length for
  dual-line) that splits C_out into N >= 4 and D >= 1 (D >= 2 with trim
  room), smallest N on a tie. A slot is BCLK low for floor(N/2) cycles
  (data changes there) and high for the rest (the DAC samples).
- UAC2, oversampled (F N D); direct formats on the fast table:

  | Rate | dual16, TDA1541A | dual18 | dual20 | dual24 | direct |
  |---|---|---|---|---|---|
  | 32 k | 16 5 5 | 20 4 5 | 20 4 5 | 25 4 4 | 64 8 9 |
  | 44.1 k | 16 9 3 | 18 4 6 | 24 6 3 | 24 6 3 | 64 8 6 |
  | 48 k | 16 5 5 | 20 4 5 | 20 4 5 | 25 4 4 | 64 8 6 |
  | 88.2 k | 16 6 3 | 18 4 4 | 24 4 3 | 24 4 3 | 64 8 3 |
  | 96 k | 16 4 6 | 24 4 4 | 24 4 4 | 24 4 4 | 64 8 3 |

  BCLK = F x output word rate: 6.144 MHz at most at F = 16 (TDA1541A
  limit 6.4 MHz, PCM56 10 MHz), up to 9.6 MHz in dual24 (fast slew on
  BCLK). The README has the BCLK table.
- Programs are generated at run time, 2 instructions per slot for N <= 16
  (`out pins, n` then nops; a group ends with `jmp x--`, `set x` or a
  nop). Direct formats: four groups (L loop of 31 slots, L last, R loop,
  R last; the special slot reloads the counter and carries the LRCK
  edge). Dual-line: the special slot 0, the loop, and for dual16 .. 24
  three LATCH slots (11 instructions; the TDA1541A 4). The side-set is
  BCLK and LRCK / LATCH (2 bits, so a 3-bit delay field).

### Formats

- Format jumper code = `i2s_format_t`: 000 I2S, 001 left-justified, 010
  right-justified 16 (24 with `PICOI2S_RJ24`; samples pre-shifted), 011
  TDA1541A simultaneous mode, 100 dual16 (PCM56), 101 dual18 (AD1865,
  AD1860 / AD1861), 110 dual20 (AD1862, PCM63, PCM1702), 111 dual24
  (PCM1704). Until 2026-10-10, 010 was right-justified 24, 011
  right-justified 16 and 100 the TDA1541A ("dual16"); right-justified 24
  became a build option to free a code for the PCM56, since nearly every
  DAC that takes it also takes I2S or left-justified.
- Dual-line: DL and DR from one `out pins, 2` (pairs interleaved by the
  table-driven `i2s_pack_dual()`), common BCLK, data MSB first,
  right-justified to the DAC's word length (the word ends in slot F-1,
  leading slots carry zeros), truncated, no dither. The word length must
  match the DAC exactly: a DAC keeps the last bits shifted in, so an
  AD1865 in dual20 drops the top two (+12 dB, wrapping above -12 dBFS).
- Dual16 .. dual24 (two's complement, data on the rising edge, transfer on
  the falling LATCH edge): LATCH rises with slot F-3 (`I2S_LATCH_SLOTS`;
  only the falling edge matters, and the datasheets ask for at most one
  period of high time, the PCM1704 for more than one) and falls one
  state-machine cycle (`I2S_LATCH_TAIL`, D clk_sys) into the BCLK-low half
  of the next frame's slot 0, before the edge that clocks the next MSB.
  Over both clock tables, oversampled or not, that leaves at least 78 ns
  from the LSB's rising edge to the falling edge (AD1860 60 ns, AD1865
  40 ns; falling with the BCLK edge it was 52 ns at the shortest) and
  26 ns from there to the next rising edge (15 ns asked); worst case
  dual24 at 48 kHz, F25 N4 D4. Datasheet notes per DAC are in
  `i2s_format.h`.
- TDA1541A simultaneous mode: offset binary, transfer on the rising LE
  edge, no clock edge while LE is high, so the pulse fills the BCLK-low
  half of slot 0 of the next frame (the first pulse latches zeros).

### Data path and filters

- Direct formats and NOS convert in the S/PDIF interrupt
  (`companion_fill`). Filtered formats (and every format while the input
  carries emphasis) copy the block into a 4-slot mailbox; core 1
  (`i2s_tx_core1_task()`) filters, saturates and packs it into the buffer
  after that: one block of lag plus the group delay. A block that finds
  its buffer already playing or the mailbox full is skipped and counted
  (`i2s_tx_late()`, session token `os<late>`, self-test `L<late>`).
  Core-1 code and tables run from RAM. Measured: packer 24 cycles per
  frame at F = 16, 34..39 otherwise; core-1 load 30..40 % at 32 .. 48 kHz,
  up to 53 % at 88.2 kHz (minimum phase on 101.6 MHz), sinc 4..15 %.
- Filters (dual-line only). Sharp is the reference; every other filter
  exists for better perceived sound with minimum pre- and post-ringing
  and is judged on that at 44.1 / 48 kHz. No apodizing filter. Jumper
  code = `i2s_filter_type_t`: 000 NOS, 001 sharp, 010 slow, 011 sinc
  compensation, 100 minimum phase, 101..111 sharp. Sharp, slow and minimum
  phase oversample 8x below 50 kHz, 4x above. Cascades in `i2s_filter.h`,
  coefficients in `i2s_filter_coef.h` from `tools/i2s_filter_design.py`
  (a rerun rewrites it identically); group delays in input samples:
  - sharp: half-bands A (131 taps), B (23), C (15) at 8x, flat to
    0.4535 Fs, ripple < 0.0002 dB, 100 dB from 0.5465 Fs; at 4x H (43)
    then B, flat to 0.35 Fs, 100 dB from 0.65 Fs. Delay 36 / 13.
  - slow: S (23) then BS (51), CS (19) at 8x or B at 4x: flat to 0.25 Fs,
    -2.8 dB at 0.4535 Fs, 100 dB from 0.75 Fs; leaks between Fs/2 and
    0.75 Fs by design. Delay 13 / 8.
  - minimum phase: M, the minimum-phase factor of S^2 (23 taps), then BS /
    CS or B: the slow magnitude without pre-ringing. Delay about 9 / 4.
  - sinc: K, 15 taps at the sample rate; with the zero-order hold flat at
    -4.0 dB +-0.03 dB to 0.4535 Fs (headroom for its +3.9 dB boost).
    Delay 7.
- Engine: Q30 coefficients, 64-bit accumulation, input at 1/16 (worst
  cascade gain 7.3), intersample overs saturate. `i2s_fold()` /
  `i2s_dot()` are hand-scheduled Thumb-2, bit-exact with the C reference.

### Self-test of the I2S output

A capture state machine samples the I2S pins after every falling BCLK
edge (it reads them one clk_sys after the edge); the decoder frames the
stream with the generator's layout (F, LSB slot, LATCH slot and pulse
length, plus the LATCH tail sample, skipping a pulse cut by the capture
start). Direct formats and NOS are checked against the test pattern;
every other filter bit-exact against the window's S/PDIF samples run
through the same filter on the device. It cannot see a slot misplacement,
since it uses the generator's own layout: the scratch-only cycle-accurate
simulator covers that by feeding the generated programs to a datasheet
model of the DACs. A capture starting inside a clipped run once matched
the wrong pattern wrap; the locate key skips silent and clipped frames.

## S/PDIF input (`src/spdif/spdif_rx*.c`)

- Coax and Toslink on their own pins with pull-downs (RP2350-One GP10 /
  GP11, Pico 2 GP26 / GP27). The receiver selects the locked input, coax
  first.
- Receiver: one 26-instruction program on pio0, one state machine per
  input, 8 cycles per cell (divider clk_sys / (1024 x Fs), set per nominal
  rate and on every clk_sys change); it waits for each slot-start edge
  and records the level after it; one word per subframe (bit k = NOT(L_k
  XOR L_k+1), parity reduces to bit 3 == bit 31). Hunt for a 3-cell run of
  one polarity; restart after 8 bad subframes. Margins (model): duty-cycle
  distortion up to about 50 ns at 44.1 / 48 kHz (2.4x a Toslink module),
  22..25 ns at 96 kHz.
- DMA (endless) into an 8 KB ring per input; `spdif_rx_task()` (main loop,
  at most once per ms) checks preambles and parity, assembles the channel
  status, hunts the rate (5 ms per divider), measures it against the
  timer (lock after a clean 16 ms window within +-1200 ppm, re-check every
  16 ms at +-1500 ppm) and drops the lock after 2 ms without words or
  8 bad subframes. A subframe is used only after 8 good ones and with its
  frame and the next one good. Frames 0..5 of each block wait for the
  channel-status bits, so non-PCM (played as zeros) and emphasis apply to
  exactly their block. FIFO of 2048 frames; every discontinuity flushes it
  and bumps `spdif_rx_generation()`. Core 0: 2.3 / 2.4 % at 44.1 / 48 kHz,
  4.3 % at 96 kHz; RAM about 34 KB.
- Arbitration (`p.src`): USB has priority. The input takes the output
  3 s after the last audible USB sample (non-zero after the volume; boot
  counts as USB activity) once an input is selected with known channel
  status; meanwhile an open USB stream is drained to its set point, and
  its first audible chunk takes the output back (that chunk is dropped).
  The input gives the output up 3 s after its last frame. Every switch
  stops the output and pre-rolls the new source. A generation change is a
  stall: a relock at the same rate resumes, another rate restarts.
- Clocking, option A: the clock servo steers the output to the input in
  both USB modes, fed with the receiver FIFO level (set point 6 ms),
  started with `SPDIF_TX_START_TRIMMED | SPDIF_TX_START_EMPHASIS`.
  Bit-perfect, NOS stays unfiltered, input jitter rejected; the cost is
  the pacer's phase error. Option C, held back: an ASRC for the
  oversampling filters only (an exception to "never resample"); the
  prototype after the first 2x stage measured -113 dB THD+N with sharp at
  57 / 62 % of core 1, slow / minimum phase -109 / -111 dB at 67..77 %.
  Rejected: ASRC for every filter (NOS loses its meaning), a VCXO (pull
  range below +-1000 ppm sources), a receiver + ASRC chip (not
  bit-perfect).
- The S/PDIF output re-transmits the input bit-exact and reclocked at
  0 dB (word length 16 or 24 from the channel status), with the emphasis
  flag copied (one flag byte per ring frame).
- De-emphasis (every I2S / dual-line format): one biquad per rate
  (`i2s_deemph.h`, `tools/i2s_deemph_design.py`), minimax fit to the
  50/15 us curve (0.0037 / 0.0013 dB to 20 kHz at 44.1 / 48 kHz), DF1 Q30,
  output rounded toward zero (no limit cycle), about 60 cycles per frame.
  With emphasis possible every format runs through core 1, so a flag
  change never moves the latency; a change crossfades over one block.
- Lessons from the bring-up: the receiver FIFO fills unread while USB
  plays, so it is emptied at the switch (`spdif_rx_flush()`; an unread
  overflow is no discontinuity); a DMA ring lap resyncs under the hold
  instead of unlocking; non-PCM blocks never carry the emphasis flag; the
  receiver moved to pio0 because it raced the self-test captures on pio1's
  CTRL register from the other core.
- LEDs: the RP2350-One WS2812 shows the source heard (red USB, green
  input, yellow input with emphasis; colours set by eye in
  `board_config.h`); the Pico 2 adds LOCK (GP22) and EMPH (GP21) LEDs.

## Diagnostics and monitoring

- Diagnostic string (`tools/diag.py`, `src/diag/diag.h`): panics, hard
  faults and main-loop hangs from the previous boot plus live status,
  95 characters.
- Session string (0x23, `app/monitor.c`): streams, frames, digital
  silence age (after the volume), uptime, late filter blocks, and for the
  input `src`, `in`, `ir<rate><ppm>`, `cs<bits>[e][n]`,
  `rx<locks>/<unlocks>/<parity>/<framing>/<overflows>`. The receiver's
  ppm is against the crystal: the PLL error in UAC2, the host offset in
  UAC1 (`monitor.py`'s lock line shows the first 16 ms window, near 0 on a
  fresh UAC1 output).
- Configuration string (0x24): board, bcdDevice, build, USB mode, format,
  filter, jumper bits, fixed at boot.
- `tools/monitor.py` turns them into events; formats in `tools/README.md`.

## USB OUT re-arm

TinyUSB leaves the ISO OUT endpoint unarmed after a zero-length packet
(see `CLAUDE.md`); a UAC1 stream once died that way after a rate change.
`check_rx_halt()` re-arms it from the main loop. A main-loop pass under
about 0.9 ms loses no packet; each further ms loses one (a 1 ms splice,
frame-aligned, no underrun below about six). Verified with injected
halts: re-armed within about 1 ms, no packet lost.

## Resources

pio2: WS2812 (RP2350-One), S/PDIF, I2S, clock pin (full on the One).
pio0: receiver (2 SMs, 26 instructions) and the two trim pacers (2 SMs,
4 instructions). pio1: self-test captures. DMA: S/PDIF 2, pacers 4,
I2S 2, receiver 2 (10 of 16; 12 with the self-test). State machines on a
block shared across the cores are enabled with the atomic CTRL aliases.

## Verification record

Tools: `tools/selftest.py` with the self-test build (bit-exact S/PDIF and
I2S, 3-s stop, soaks), `tools/monitor.py`, and scratch-only harnesses:
the PIO simulator, the self-test host tests, a test transmitter on the
clock pin for the input, forced-format builds, a core-0 load meter, an
MCLK meter.

- USB, UAC2: bit-exact at all ten stream formats on S/PDIF and I2S;
  Windows 11, iPad / iPhone (Apple feedback path), Android (Eversolo T8),
  Linux (Raspberry Pi 5, Debian 13, plain ALSA: sweep ALL PASS, 120 rapid
  open / close and format switches, 5 re-enumerations, Feature Unit as
  ALSA `PCM`; ALSA starts a new card at -20 dB, so bit-perfect playback
  needs `PCM` at 0 dB).
- USB, UAC1: Windows 11 (classic driver), iPad / iPhone, Linux (trim +192 ppm on the Pi:
  its USB clock -151 ppm plus the -344 ppm PLL error), 90 rapid open /
  close and rate changes; the trim follows the host within a few ppm.
  With the trim pacer: five consecutive sweeps ALL PASS, core-0 load
  measured (see the pacer).
- Volume and ramp: Windows in both modes, iPad / iPhone, Eversolo T8.
  Latency: lip-sync judged good on Windows with video.
- I2S: bit-exact in every format and filter by self-test sweeps (forced
  formats in scratch builds, both modes); simulator and host tests for
  every format, filter and clock. Real hardware: an AD1865 in dual18 with
  every filter, and again after each LATCH change (owner's listening
  tests). MCLK measured within 4 ppm of 256 Fs x (1 + trim).
- Scope (2026-10-10, de59100): dual18 at 44.1 and 48 kHz (8x), the
  input's trimmed clock, MCLK in I2S, the DEM clock in TDA1541A, the
  S/PDIF output and the 3-s stop all as calculated. At 44.1 kHz: BCLK
  6.3507 MHz, 50 % duty, data changes with the falling edge (75 ns setup
  and hold), LATCH 514 ns high, falling 40 ns after the BCLK fall and
  120 ns after the LSB edge. The -0.9 V undershoot first seen on BCLK
  and data came from the setup (long jumper wires, probe ground leads);
  with short wiring, proper grounding and a probe spring there is almost
  none.
- Rename to PicoI2S (new USB strings, output terminal type 0x0602):
  self-test sweeps in both modes on Windows and Linux (ALSA card id
  `I2S`, names "Pico USB to I2S" / "Pico I2S", Feature Unit still `PCM`),
  music and volume on iOS in both modes. Android not yet re-checked.
- Receiver alone (both state machines reading the transmitter's pin): 6
  clk_sys x 5 rates x +-1000 ppm bit-exact, an 8.4 M-frame soak, lock
  about 18 ms, unlock 7..8 ms, coax takeover 18..35 ms, clock switches
  keep the lock.
- S/PDIF input: end to end with a test transmitter (5 rates x +-1000 ppm
  in both modes, emphasis, non-PCM, dropouts, rate changes, all
  arbitration cases; lock 18..29 ms, take-back 22..36 ms, output off at
  3.0 s); a real Eversolo source on Toslink (44.1 / 48 / 96 kHz, rate
  changes relock within a second, zero errors, its clock -102 ppm); the
  board's own Toslink output looped into its input.
- Soaks, all with zero events (no underrun, overrun, late interrupt or
  late filter block):

  | Firmware | Mode | Format | Length | Notes |
  |---|---|---|---|---|
  | pipeline (phase B) | UAC2 | Android playback | 2 h | Eversolo T8 host, clean |
  | volume | UAC2 | 96/24 | 4 h | |
  | volume | UAC1 | 44.1/16 | 4 h | trim +332..+334 ppm, locked in all 480 reads |
  | Linux bring-up | UAC2 | 44.1/16, 96/24 | 9 + 9 min | Raspberry Pi 5 |
  | Linux bring-up | UAC1 | 44.1/16 | 9 min | Raspberry Pi 5, trim +192 ppm |
  | S/PDIF input | UAC2 / UAC1 | input cycling | 15 + 20 min | test transmitter |
  | trim pacer | UAC1 | input cycling | 8 min | test transmitter |
  | ccc06f8 | UAC2 | 96/24 | 5 min | Linux, dual18 minimum phase |
  | ccc06f8 | UAC1 | 44.1/16 | 5 min | Linux, trim +192..+197 ppm |
  | ccc06f8 | UAC2 | 44.1/16, 48/24, 96/24 | 5 min each | optical loop |
  | ccc06f8 | UAC1 | 44.1/16 | 2 h | optical loop, trim +233..+236 ppm, locked in all 1440 reads |
  | ccc06f8 | UAC2 | 44.1/16 | 2 h | optical loop |
  | ccc06f8 | UAC2 | 96/24 | 2 h | optical loop |

  The optical-loop soaks (2026-10-10, overnight) ran with the input
  locked on the board's own output and counted zero parity, framing and
  overflow errors over the optical link.

## Known limitations

- A rate change without a signal gap on the input (44.1 <-> 48) plays a
  few ms of new-rate frames at the old rate before the 16 ms re-check
  unlocks, and counts a few hundred receiver errors.
- The take-back from the input drops the first audible USB chunk (up to
  2 ms).
- The silence tail after an emphasized input keeps the emphasis flag.
- With the output looped into an input and a USB stream open but silent,
  the output cycles every 3 s (the input takes it, which stops its own
  source). A test setup only.
- The trimmed modes carry the pacer's phase error on the I2S clocks (see
  the trim pacer).
- After a flash with picotool 2.1.1 (Debian; two-step reflash) the
  previous-boot record reads `? m2 pc10000000`: the boot ROM's
  flash-update reboot keeps its parameters in watchdog scratch 2 and 3,
  which the diagnostics use. Not a crash.
