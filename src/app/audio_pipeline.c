#include "app/audio_pipeline.h"

#include "pico.h"
#include "pico/time.h"

#include "app/clock_servo.h"
#include "app/log.h"
#include "clock/audio_clock.h"
#include "diag/diag.h"
#include "spdif/spdif_rx.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"

// Ring of stereo frames: the main loop produces, the S/PDIF DMA interrupt
// consumes. Indices are free-running frame counts. Each frame carries the
// SPDIF_TX_FLAG_* bits of its source chunk.
#define RING_FRAMES   512u
#define RING_MASK     (RING_FRAMES - 1u)
// Topped up to this level only, so most of the buffering stays in the USB
// FIFO, where the feedback endpoint (UAC2) or the clock servo (UAC1)
// regulates it. It only has to bridge main-loop stalls: 2 ms at 96 kHz,
// and at least two DMA buffers.
#define RING_TARGET_MIN  192u
#define RING_TARGET   (2u * SPDIF_TX_BUFFER_FRAMES > RING_TARGET_MIN ? \
                       2u * SPDIF_TX_BUFFER_FRAMES : RING_TARGET_MIN)
// Frames held by the transmitter's two DMA buffers.
#define DMA_FRAMES    (2u * SPDIF_TX_BUFFER_FRAMES)
#define READ_FRAMES   SPDIF_TX_BUFFER_FRAMES
#define IDLE_STOP_US  3000000u
// USB gives the output up to the input after this long without an audible
// sample.
#define USB_IDLE_US   3000000u
// S/PDIF input FIFO: 32-bit stereo frames, held at the USB set point in
// time (FIFO_SETPOINT_PACKETS ms).
#define INPUT_FRAME_BYTES 8u
// Ramped attenuation value for silence (mute): one step below -64 dB.
#define ATT_SILENT    (USB_AUDIO_ATT_MAX_DB + 1u)
// A source switch fades the outgoing source out and the incoming one in,
// this many dB per chunk: 11 chunks between silence and 0 dB, 22 ms at
// 48 kHz.
#define FADE_DB       6u
// The take-back switches once the faded-out input has left the DMA buffers
// and the I2S path (one block plus the filter's group delay).
#define HANDOVER_FRAMES (DMA_FRAMES + 2u * SPDIF_TX_BUFFER_FRAMES)

_Static_assert((RING_FRAMES & RING_MASK) == 0, "ring size must be a power of two");
_Static_assert(RING_TARGET <= RING_FRAMES, "ring target exceeds the ring");

static int32_t ring[RING_FRAMES * 2];
static uint8_t ring_flags[RING_FRAMES];
static volatile uint32_t ring_wr;      // written by the main loop only
static volatile uint32_t ring_rd;      // written by the interrupt only
static volatile bool ring_short;       // the interrupt found too few frames
static volatile uint8_t ring_out_flags; // flags of the last block sent

static union {
    int16_t s16[READ_FRAMES * 2];
    int32_t s32[READ_FRAMES * 2];
} rx;

typedef enum {
    SRC_USB = 0,
    SRC_INPUT,
} source_t;

typedef struct {
    source_t src;             // owner of the ring and the output
    uint64_t usb_heard_us;    // an audible USB sample last arrived
    bool     open;            // stream open at the last poll
    uint32_t rate;            // format of the open stream
    uint8_t  bits;
    bool     playing;
    bool     hold;            // output failed to start: wait for the next stream
    bool     first_play;      // not played yet since the stream started
    bool     loop_reset;      // reset the loop maximum next iteration
    uint64_t last_audio_us;   // stream samples last moved into the ring
    uint64_t last_signal_us;  // a non-zero sample last moved into the ring
    bool     had_signal;
    uint32_t streams;
    uint32_t frames;
    uint8_t  att;             // attenuation applied to the ring, 0 .. ATT_SILENT
    bool     fade_in;         // the next USB start fades in (take-back)
    uint32_t start_failures;
    uint32_t overruns_base;   // counts when the stream started
    uint32_t late_irqs_base;
    audio_pipeline_stats_t st;
} pipeline_t;

static pipeline_t p;

// S/PDIF input while it owns the output.
typedef struct {
    spdif_rx_status_t st;     // receiver status, polled every iteration
    uint32_t gen;             // receiver generation last seen
    spdif_rx_input_t input;   // input whose frames go into the ring
    uint32_t rate;            // rate the clock and output are set up for, 0: none
    bool     fresh;           // input changed: restart the servo, not resume it
    bool     hold;            // the output failed to start at generation hold_gen
    uint32_t hold_gen;
    bool     handover;        // fading out for the USB take-back
    bool     faded;           // silent from ring index faded_wr on
    uint32_t faded_wr;
} input_t;

static input_t in;

// UAC1 or the S/PDIF input: steers the output rate to the source's. Runs
// while playing.
static clock_servo_t servo;

static bool adaptive(void) {
    return usb_audio_mode() == USB_AUDIO_MODE_UAC1;
}

// The interrupt runs to completion before the main loop resumes, so only
// the producer side needs ordering. A short ring plays what it has, then
// silence.
static uint32_t __not_in_flash_func(ring_source)(int32_t *pcm, uint32_t n_frames) {
    uint32_t rd = ring_rd;
    uint32_t n = ring_wr - rd;
    if (n > n_frames) {
        n = n_frames;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t k = ((rd + i) & RING_MASK) * 2u;
        pcm[2u * i]      = ring[k];
        pcm[2u * i + 1u] = ring[k + 1u];
    }
    // The block takes the flags of its middle frame (a chunk with other
    // flags may start inside it); silence keeps the last.
    if (n > 0) {
        ring_out_flags = ring_flags[(rd + n / 2u) & RING_MASK];
    }
    ring_rd = rd + n;
    if (n < n_frames) {
        // volatile: stops the compiler from turning this into a memset call,
        // which lives in flash.
        volatile int32_t *out = pcm;
        for (uint32_t i = 2u * n; i < 2u * n_frames; i++) {
            out[i] = 0;
        }
        ring_short = true;
    }
    return ring_out_flags;
}

// Only while the transmitter is stopped (the interrupt owns ring_rd).
static void ring_flush(void) {
    ring_rd = ring_wr;
    ring_out_flags = 0;
}

static void ring_set_flags(uint32_t wr, uint32_t n, uint8_t flags) {
    for (uint32_t i = 0; i < n; i++) {
        ring_flags[(wr + i) & RING_MASK] = flags;
    }
}

// Host volume (Feature Unit), applied here in the main loop only. Q31 gain
// per dB of attenuation: round(2^31 x 10^(-k/20)), 0 dB saturated.
static const int32_t gain_q31[] = {
    2147483647, 1913946816, 1705806895, 1520301996, 1354970580,
    1207618800, 1076291389,  959245710,  854928639,  761955951,
     679093957,  605243126,  539423504,  480761704,  428479319,
     381882595,  340353221,  303340128,  270352174,  240951628,
     214748365,  191394682,  170580690,  152030200,  135497058,
     120761880,  107629139,   95924571,   85492864,   76195595,
      67909396,   60524313,   53942350,   48076170,   42847932,
      38188260,   34035322,   30334013,   27035217,   24095163,
      21474836,   19139468,   17058069,   15203020,   13549706,
      12076188,   10762914,    9592457,    8549286,    7619560,
       6790940,    6052431,    5394235,    4807617,    4284793,
       3818826,    3403532,    3033401,    2703522,    2409516,
       2147484,    1913947,    1705807,    1520302,    1354971,
};

_Static_assert(sizeof(gain_q31) / sizeof(gain_q31[0]) == USB_AUDIO_ATT_MAX_DB + 1u,
               "one gain per dB of the volume range");

// Scales a left-justified sample and rounds the exact product once, to the
// 24 bits the encoder sends (bits 31..8). Only used below 0 dB, where the
// result cannot overflow, so no saturation.
static int32_t scale_sample(int32_t sample, int32_t gain) {
    int64_t y = ((int64_t)sample * gain + (1ll << 38)) >> 39;
    return (int32_t)y * 256;
}

// Attenuation the host asks for; mute is ATT_SILENT.
static uint8_t att_target(usb_audio_volume_t v) {
    return v.mute ? (uint8_t)ATT_SILENT : v.att_db;
}

// Moves the applied attenuation up to step dB towards the target, once per
// chunk (96 frames when steady, 2 ms at 48 kHz). The host volume moves 1 dB
// per chunk, so a jump or a mute fades as on the PCM2704.
static void att_step(uint8_t target, uint8_t step) {
    if (p.att < target) {
        p.att = (uint8_t)(target - p.att > step ? p.att + step : target);
    } else if (p.att > target) {
        p.att = (uint8_t)(p.att - target > step ? p.att - step : target);
    }
}

static int32_t att_gain(uint8_t att) {
    return att == ATT_SILENT ? 0 : gain_q31[att];
}

// Attenuates or mutes n frames just written at ring index wr, the gain
// moving linearly from att0's to att1's across them, so a ramp step has no
// edge.
static void apply_volume(uint32_t wr, uint32_t n, uint8_t att0, uint8_t att1) {
    int32_t gain = att_gain(att0);
    int32_t step = (att_gain(att1) - gain) / (int32_t)n;
    for (uint32_t i = 0; i < n; i++) {
        gain += step;
        uint32_t k = ((wr + i) & RING_MASK) * 2u;
        ring[k]      = scale_sample(ring[k], gain);
        ring[k + 1u] = scale_sample(ring[k + 1u], gain);
    }
}

// Ramps the n frames just written at ring index wr one step towards the
// target attenuation. Untouched at 0 dB (bit-perfect).
static void ramp_chunk(uint32_t wr, uint32_t n, uint8_t target, uint8_t step) {
    uint8_t att0 = p.att;
    att_step(target, step);
    if (att0 != 0 || p.att != 0) {
        apply_volume(wr, n, att0, p.att);
    }
}

// S/PDIF word length: attenuated samples carry 24 significant bits,
// whatever the stream's depth. Follows the applied attenuation, i.e. the
// samples in the ring.
static uint8_t output_word_bits(uint8_t resolution) {
    return p.att != 0 ? 24u : resolution;
}

// Channel-status word length of a running output, from the stream depth
// and the applied volume (output_word_bits()). Updated in place: a volume
// change never restarts the output. The 24 -> 16 switch leads the
// attenuated audio still in the ring and DMA buffers (up to 12 ms), which
// is harmless: the word length only tells the receiver how many bits are
// significant.
static void sync_word_bits(void) {
    if (!spdif_tx_running() || p.bits == 0) {
        return;
    }
    uint8_t bits = output_word_bits(p.bits);
    if (spdif_tx_word_bits() != bits) {
        spdif_tx_set_word_bits(bits);
    }
}

// True if any of the n frames at ring index wr is non-zero.
static bool chunk_has_signal(uint32_t wr, uint32_t n) {
    int32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t k = ((wr + i) & RING_MASK) * 2u;
        acc |= ring[k] | ring[k + 1u];
    }
    return acc != 0;
}

// Moves stream frames into the ring, up to RING_TARGET. Returns the count.
// Each read is a chunk of up to READ_FRAMES with one ramp step. At 0 dB the
// samples are copied unchanged (bit-perfect).
static uint32_t pump(const usb_audio_format_t *f, uint64_t now) {
    uint32_t frame_bytes = 2u * f->subslot_bytes;
    uint32_t moved = 0;
    uint8_t target = att_target(usb_audio_volume());
    for (;;) {
        uint32_t wr = ring_wr;
        uint32_t level = wr - ring_rd;
        if (level >= RING_TARGET) {
            break;
        }
        uint32_t n = RING_TARGET - level;
        if (n > READ_FRAMES) {
            n = READ_FRAMES;
        }
        n = usb_audio_read(rx.s32, n * frame_bytes) / frame_bytes;
        if (n == 0) {
            break;
        }
        if (f->subslot_bytes == 2u) {
            for (uint32_t i = 0; i < n; i++) {
                uint32_t k = ((wr + i) & RING_MASK) * 2u;
                ring[k]      = (int32_t)rx.s16[2u * i] * 65536;
                ring[k + 1u] = (int32_t)rx.s16[2u * i + 1u] * 65536;
            }
        } else {
            // 24-bit samples arrive left-justified in 32-bit subslots.
            for (uint32_t i = 0; i < n; i++) {
                uint32_t k = ((wr + i) & RING_MASK) * 2u;
                ring[k]      = rx.s32[2u * i];
                ring[k + 1u] = rx.s32[2u * i + 1u];
            }
        }
        ramp_chunk(wr, n, target, p.fade_in ? FADE_DB : 1u);
        if (p.att == target) {
            p.fade_in = false;
        }
        ring_set_flags(wr, n, 0);
        // What goes out on the line: a muted stream is digital silence.
        if (chunk_has_signal(wr, n)) {
            p.last_signal_us = now;
            p.had_signal = true;
            p.usb_heard_us = now;
        }
        // Before the chunk is published, so 24 bits are flagged by the time
        // the first attenuated samples go out.
        sync_word_bits();
        __compiler_memory_barrier();
        ring_wr = wr + n;
        moved += n;
    }
    p.frames += moved;
    return moved;
}

static void top_up(const usb_audio_format_t *f, uint64_t now) {
    if (pump(f, now) > 0) {
        p.last_audio_us = now;
    }
}

static void output_stop(void) {
    spdif_tx_stop();
    ring_flush();
    p.playing = false;
    log_printf("spdif: stop\n");
}

// Output stopped: switches clk_sys if sample_rate needs another setting.
static void clock_follow(uint32_t sample_rate) {
    if (audio_clock_rate_changes(sample_rate)) {
        uint32_t hz = audio_clock_set_rate(sample_rate);
        diag_clock_changed();
        log_printf("clock: %lu Hz\n", (unsigned long)hz);
    }
}

// Starts the stopped output from the ring (options: SPDIF_TX_START_*). On
// failure the ring is flushed; the caller holds off retrying.
static bool output_start(uint32_t sample_rate, uint8_t bits, uint32_t options) {
    if (!spdif_tx_start(sample_rate, bits, ring_source, options)) {
        p.start_failures++;
        ring_flush();
        log_printf("spdif: start %lu failed (%lu)\n", (unsigned long)sample_rate,
                   (unsigned long)p.start_failures);
        return false;
    }
    log_printf("spdif: %lu Hz %u-bit\n", (unsigned long)sample_rate, bits);
    return true;
}

// FIFO level held by the feedback (UAC2) or the clock servo (UAC1).
static uint32_t fifo_setpoint(const usb_audio_format_t *f) {
    return usb_audio_fifo_setpoint(f->sample_rate, f->subslot_bytes);
}

// FIFO level that starts playback. The start moves RING_TARGET + DMA_FRAMES
// out of the FIFO, so waiting for that much above the set point leaves the
// FIFO there and the stream starts with its steady-state buffering. Capped
// two packets below full: a packet or two may arrive between checks, and a
// full FIFO overwrites.
// A running output does not take its two DMA buffers from the FIFO. The
// UAC2 feedback absorbs those extra frames; the UAC1 clock servo only
// holds a starting level within two chunks of the set point, so there they
// are left out.
static uint32_t preroll_bytes(const usb_audio_format_t *f, uint32_t depth) {
    uint32_t frame_bytes = 2u * f->subslot_bytes;
    uint32_t moved = RING_TARGET + (adaptive() && spdif_tx_running() ? 0u : DMA_FRAMES);
    uint32_t want = fifo_setpoint(f) + moved * frame_bytes;
    uint32_t packet = ((f->sample_rate + 999u) / 1000u + 1u) * frame_bytes;
    uint32_t cap = depth - 2u * packet;
    return want < cap ? want : cap;
}

static void stats_reset(void) {
    usb_audio_stats_t us;
    usb_audio_get_stats(&us);
    p.overruns_base = us.overruns;
    p.late_irqs_base = spdif_tx_late_irqs();
    p.st.underruns = 0;
    p.st.ring_min = RING_TARGET;
    usb_audio_fifo_max_reset();
    diag_loop_max_reset();
    p.first_play = true;
}

// Stream opened, or its format changed while open. Event bits carry no
// order, so this is derived from the state. While the input owns the
// output, the stream is only drained.
static void stream_started(const usb_audio_format_t *f, uint64_t now) {
    p.rate = f->sample_rate;
    p.bits = f->resolution;
    p.hold = false;
    p.streams++;
    if (p.src != SRC_USB) {
        return;
    }
    p.playing = false;
    p.fade_in = false;
    stats_reset();
    // An opening stream counts as audio, so a running output is not stopped
    // during its pre-roll.
    p.last_audio_us = now;
    // Same rate, other depth: sync_word_bits() updates the running output.
    if (spdif_tx_running() && spdif_tx_sample_rate() != f->sample_rate) {
        output_stop();
    }
}

// A fresh output starts from the nominal rate (its known clock error
// removed); a running one resumes the servo, so a restart continues from
// the locked rate (or from the current trim) without a jump.
static void servo_start(uint32_t sample_rate, uint32_t frame_bytes, uint32_t setpoint_bytes,
                        bool fresh) {
    clock_servo_params_t sp = {
        .sample_rate = sample_rate,
        .frame_bytes = frame_bytes,
        .setpoint_bytes = setpoint_bytes,
        .chunk_frames = SPDIF_TX_BUFFER_FRAMES,
        .trim_ppb = fresh ? -spdif_tx_rate_error_ppb() : spdif_tx_trim_ppb(),
    };
    if (fresh) {
        clock_servo_start(&servo, &sp);
    } else {
        clock_servo_resume(&servo, &sp);
    }
    spdif_tx_set_trim_ppb(clock_servo_trim_ppb(&servo));
}

static void servo_update(uint32_t level_bytes, uint64_t now) {
    if (clock_servo_update(&servo, level_bytes, now)) {
        spdif_tx_set_trim_ppb(clock_servo_trim_ppb(&servo));
    }
}

// The ring is filled and the output runs: the source is heard.
static void play_begun(void) {
    // After the fill: a flag raised before it is stale.
    ring_short = false;
    p.playing = true;
    if (p.first_play) {
        // Keep the start-up (pre-roll peak, clock switch, pre-fill) out of
        // the maxima. This iteration is only measured at the next
        // diag_loop(), hence the deferred loop reset.
        p.first_play = false;
        usb_audio_fifo_max_reset();
        p.loop_reset = true;
    }
}

// Pre-roll done: fill the ring and start the output if it is stopped.
static void begin_playing(const usb_audio_format_t *f, uint64_t now) {
    bool start = !spdif_tx_running();
    // Silence precedes it (pre-roll, underrun or stopped output), so it
    // starts at the requested volume, unless it takes the output back from
    // the input in the middle of its signal.
    p.att = p.fade_in ? (uint8_t)ATT_SILENT : att_target(usb_audio_volume());
    if (start) {
        clock_follow(f->sample_rate);
    }
    top_up(f, now);
    if (start) {
        uint32_t options = adaptive() ? SPDIF_TX_START_TRIMMED : 0u;
        if (!output_start(f->sample_rate, output_word_bits(f->resolution), options)) {
            p.hold = true;
            return;
        }
        top_up(f, now);   // the start pre-filled two buffers from the ring
    }
    if (adaptive()) {
        servo_start(f->sample_rate, 2u * f->subslot_bytes, fifo_setpoint(f), start);
    }
    play_begun();
}

static void ring_track_min(void) {
    // Lowest point: only the interrupt has run since the last top-up.
    uint32_t level = ring_wr - ring_rd;
    if (level < p.st.ring_min) {
        p.st.ring_min = level;
    }
}

// The USB stream owns the output.
static void usb_task(bool open, const usb_audio_format_t *f, uint64_t now) {
    if (open && !p.playing && !p.hold) {
        usb_audio_stats_t us;
        usb_audio_get_stats(&us);
        if (us.fifo_level >= preroll_bytes(f, us.fifo_depth)) {
            begin_playing(f, now);
        }
    }

    if (p.playing) {
        ring_track_min();
        top_up(f, now);
        // Only while playing. Otherwise (underrun, stream closed) the trim
        // stays, and the next begin_playing() resumes the servo.
        if (adaptive()) {
            servo_update(usb_audio_fifo_level(), now);
        }
    }
}

// True if the n frames just read from the USB FIFO (rx) would be audible
// after the host's volume.
static bool usb_chunk_audible(const usb_audio_format_t *f, uint32_t n, uint8_t att) {
    if (att == ATT_SILENT) {
        return false;
    }
    for (uint32_t i = 0; i < 2u * n; i++) {
        int32_t s = f->subslot_bytes == 2u ? (int32_t)rx.s16[i] * 65536 : rx.s32[i];
        if (att != 0 && s != 0) {
            s = scale_sample(s, gain_q31[att]);
        }
        if (s != 0) {
            return true;
        }
    }
    return false;
}

// While the input owns the output: reads the open USB stream down to the
// FIFO set point and discards it, so it neither overruns nor delays a
// take-back. Stops at the first audible chunk and returns true.
static bool usb_drain(const usb_audio_format_t *f) {
    uint32_t frame_bytes = 2u * f->subslot_bytes;
    uint32_t setpoint = fifo_setpoint(f);
    uint8_t att = att_target(usb_audio_volume());
    for (;;) {
        uint32_t level = usb_audio_fifo_level();
        uint32_t n = level > setpoint ? (level - setpoint) / frame_bytes : 0u;
        if (n > READ_FRAMES) {
            n = READ_FRAMES;
        }
        if (n == 0) {
            return false;
        }
        n = usb_audio_read(rx.s32, n * frame_bytes) / frame_bytes;
        if (n == 0) {
            return false;
        }
        if (usb_chunk_audible(f, n, att)) {
            return true;
        }
    }
}

static audio_source_t input_source(spdif_rx_input_t input) {
    switch (input) {
    case SPDIF_RX_COAX:    return AUDIO_SOURCE_COAX;
    case SPDIF_RX_TOSLINK: return AUDIO_SOURCE_TOSLINK;
    default:               return AUDIO_SOURCE_NONE;
    }
}

static const char *input_name(spdif_rx_input_t input) {
    return input == SPDIF_RX_COAX ? "coax" : (input == SPDIF_RX_TOSLINK ? "toslink" : "none");
}

// A selected, locked input with a known channel status that has not failed
// to start since its last discontinuity.
static bool input_ready(void) {
    return in.st.input != SPDIF_RX_NONE && in.st.cs_valid && in.st.sample_rate != 0 &&
           !(in.hold && in.gen == in.hold_gen);
}

// Receiver FIFO level held by the clock servo, in frames.
static uint32_t input_setpoint(uint32_t sample_rate) {
    return FIFO_SETPOINT_PACKETS * ((sample_rate + 999u) / 1000u);
}

// As preroll_bytes(), in frames: a running output leaves out the DMA
// buffers (the servo holds the starting level), and the cap leaves room for
// what arrives between checks (an overflow flushes the receiver FIFO).
static uint32_t input_preroll_frames(uint32_t sample_rate) {
    uint32_t moved = RING_TARGET + (spdif_tx_running() ? 0u : DMA_FRAMES);
    uint32_t want = input_setpoint(sample_rate) + moved;
    uint32_t margin = 2u * ((sample_rate + 999u) / 1000u + 1u);
    uint32_t depth = spdif_rx_depth();
    uint32_t cap = depth > margin ? depth - margin : 0u;
    return want < cap ? want : cap;
}

static uint8_t input_word_bits(void) {
    return in.st.word_bits <= 16u ? 16u : 24u;
}

// Moves input frames into the ring, up to RING_TARGET, each chunk with its
// emphasis flag. Unchanged (bit-perfect) except while fading in after a
// start or out for the take-back. Returns the count.
static uint32_t pump_input(uint64_t now) {
    uint32_t moved = 0;
    for (;;) {
        uint32_t wr = ring_wr;
        uint32_t level = wr - ring_rd;
        if (level >= RING_TARGET) {
            break;
        }
        uint32_t n = RING_TARGET - level;
        if (n > READ_FRAMES) {
            n = READ_FRAMES;
        }
        // Read in place, up to the end of the ring: the interrupt does not
        // read past ring_wr.
        uint32_t room = RING_FRAMES - (wr & RING_MASK);
        if (n > room) {
            n = room;
        }
        uint32_t rx_flags;
        n = spdif_rx_read(&ring[(wr & RING_MASK) * 2u], n, &rx_flags);
        if (n == 0) {
            break;
        }
        ring_set_flags(wr, n, (rx_flags & SPDIF_RX_FLAG_EMPHASIS) ? SPDIF_TX_FLAG_EMPHASIS : 0u);
        ramp_chunk(wr, n, in.handover ? (uint8_t)ATT_SILENT : 0u, FADE_DB);
        if (in.handover && !in.faded && p.att == ATT_SILENT) {
            in.faded = true;
            in.faded_wr = wr + n;
        }
        if (chunk_has_signal(wr, n)) {
            p.last_signal_us = now;
            p.had_signal = true;
        }
        __compiler_memory_barrier();
        ring_wr = wr + n;
        moved += n;
    }
    p.frames += moved;
    return moved;
}

static void top_up_input(uint64_t now) {
    if (pump_input(now) > 0) {
        p.last_audio_us = now;
    }
}

// The input's rate is new (or it just took the output): an output at
// another rate stops, and clk_sys switches before the pre-roll, which the
// receiver then refills at the new clock.
static void input_set_rate(uint32_t sample_rate) {
    if (spdif_tx_running() && spdif_tx_sample_rate() != sample_rate) {
        output_stop();
    }
    if (!spdif_tx_running()) {
        clock_follow(sample_rate);
    }
    in.rate = sample_rate;
}

// Pre-roll done: as begin_playing(), with the output trimmed to the input
// in either USB mode.
static void begin_input(uint64_t now) {
    bool start = !spdif_tx_running();
    // The input is joined in the middle of its signal: fade in.
    p.att = ATT_SILENT;
    top_up_input(now);
    if (start) {
        if (!output_start(in.rate, input_word_bits(),
                          SPDIF_TX_START_TRIMMED | SPDIF_TX_START_EMPHASIS)) {
            in.hold = true;
            in.hold_gen = in.gen;
            return;
        }
        top_up_input(now);
    }
    servo_start(in.rate, INPUT_FRAME_BYTES, input_setpoint(in.rate) * INPUT_FRAME_BYTES,
                start || in.fresh);
    in.fresh = false;
    play_begun();
}

// Polls the receiver. A discontinuity is a stall, not an underrun: the
// ring drains, then the output sends silence.
static void input_poll(void) {
    spdif_rx_get_status(&in.st);
    uint32_t gen = spdif_rx_generation();
    if (gen != in.gen) {
        in.gen = gen;
        if (p.src == SRC_INPUT && p.playing) {
            p.playing = false;
            log_printf("pipe: input discontinuity\n");
        }
    }
}

// The S/PDIF input owns the output.
static void input_task(uint64_t now) {
    if (!input_ready()) {
        p.playing = false;
        return;
    }
    if (in.st.input != in.input) {
        in.input = in.st.input;
        in.fresh = true;
    }
    if (in.st.sample_rate != in.rate) {
        input_set_rate(in.st.sample_rate);
    }
    if (spdif_tx_running() && spdif_tx_word_bits() != input_word_bits()) {
        spdif_tx_set_word_bits(input_word_bits());
    }

    if (!p.playing && spdif_rx_level() >= input_preroll_frames(in.rate)) {
        begin_input(now);
    }

    if (p.playing) {
        ring_track_min();
        top_up_input(now);
        servo_update(spdif_rx_level() * INPUT_FRAME_BYTES, now);
    }
}

static void log_stream_end(void) {
    audio_pipeline_stats_t st;
    audio_pipeline_get_stats(&st);
    log_printf("pipe: end ur%lu ov%lu lt%lu rmin%lu fmax%lu lp%lu tr%ld\n",
               (unsigned long)st.underruns, (unsigned long)st.overruns,
               (unsigned long)st.late_irqs, (unsigned long)st.ring_min,
               (unsigned long)st.fifo_max, (unsigned long)st.loop_max_us,
               (long)st.trim_ppb);
}

// Hands the output to the other source: stop, then the new one pre-rolls.
static void switch_source(source_t src, uint64_t now) {
    if (!p.first_play) {
        log_stream_end();
    }
    if (spdif_tx_running()) {
        output_stop();
    }
    p.src = src;
    p.playing = false;
    // The receiver fills its FIFO while unread: start the pre-roll empty.
    if (src == SRC_INPUT) {
        spdif_rx_flush();
    }
    // Counts as audio, so the input gets 3 s to pre-roll.
    p.last_audio_us = now;
    in.input = SPDIF_RX_NONE;
    in.rate = 0;
    in.handover = false;
    stats_reset();
}

// The faded-out input has played out on every output.
static bool input_faded(void) {
    return in.faded && ring_rd - in.faded_wr >= HANDOVER_FRAMES;
}

// USB has priority: the input takes the output after USB_IDLE_US without
// an audible USB sample, and the first one takes it back once the input
// has faded out. The USB stream is drained meanwhile, then fades in.
static void arbitrate(bool open, const usb_audio_format_t *f, uint64_t now) {
    if (p.src == SRC_INPUT) {
        if (open && usb_drain(f) && !in.handover) {
            p.usb_heard_us = now;
            in.handover = true;
            in.faded = false;
        }
        if (in.handover && (!p.playing || input_faded())) {
            log_printf("pipe: source usb\n");
            switch_source(SRC_USB, now);
            p.fade_in = true;
        }
    } else if (input_ready() && now - p.usb_heard_us >= USB_IDLE_US) {
        log_printf("pipe: source %s %lu Hz\n", input_name(in.st.input),
                   (unsigned long)in.st.sample_rate);
        switch_source(SRC_INPUT, now);
    }
}

void audio_pipeline_init(void) {
    ring_wr = 0;
    ring_rd = 0;
    ring_short = false;
    ring_out_flags = 0;
    p = (pipeline_t){0};
    in = (input_t){0};
    // Boot counts as USB activity: the input waits USB_IDLE_US for a host.
    p.usb_heard_us = time_us_64();
}

void audio_pipeline_task(void) {
    uint64_t now = time_us_64();
    bool open = usb_audio_link_state() == USB_AUDIO_STREAMING;
    usb_audio_format_t f = usb_audio_format();

    if (p.loop_reset) {
        p.loop_reset = false;
        diag_loop_max_reset();
    }

    input_poll();

    bool changed = f.sample_rate != p.rate || f.resolution != p.bits;
    if (p.src == SRC_USB && p.open && (!open || changed)) {
        log_stream_end();
    }
    if (open && (!p.open || changed)) {
        stream_started(&f, now);
    }
    p.open = open;
    if (p.src == SRC_USB && !open) {
        // The ring drains, then the output sends silence.
        p.playing = false;
    }

    arbitrate(open, &f, now);

    if (p.src == SRC_USB) {
        // Follows a change of the stream depth; volume changes are synced
        // per chunk in pump().
        sync_word_bits();
    }

    if (ring_short) {
        ring_short = false;
        if (p.playing) {
            p.st.underruns++;
            p.st.underruns_total++;
            p.st.ring_min = 0;
            p.playing = false;
            log_printf("pipe: underrun (%lu)\n", (unsigned long)p.st.underruns_total);
        }
    }

    if (p.src == SRC_USB) {
        usb_task(open, &f, now);
    } else {
        input_task(now);
    }

    if (now - p.last_audio_us >= IDLE_STOP_US) {
        if (spdif_tx_running()) {
            output_stop();
        }
        if (p.src == SRC_INPUT) {
            log_printf("pipe: source usb (input idle)\n");
            switch_source(SRC_USB, now);
        }
    }
}

bool audio_pipeline_playing(void) {
    return p.playing && p.src == SRC_USB;
}

void audio_pipeline_get_stats(audio_pipeline_stats_t *stats) {
    usb_audio_stats_t us;
    usb_audio_get_stats(&us);
    *stats = p.st;
    stats->overruns = us.overruns - p.overruns_base;
    stats->overruns_total = us.overruns;
    stats->late_irqs = spdif_tx_late_irqs() - p.late_irqs_base;
    stats->fifo_max = us.fifo_max;
    stats->loop_max_us = diag_loop_max_us();
    stats->trim_ppb = spdif_tx_trim_ppb();
    stats->clock_locked = p.playing && (p.src == SRC_INPUT || adaptive()) &&
                          clock_servo_locked(&servo);
}

void audio_pipeline_get_session(audio_pipeline_session_t *session) {
    session->streams = p.streams;
    session->frames = p.frames;
    session->signal_age_ms = UINT32_MAX;
    if (p.had_signal) {
        uint64_t age = (time_us_64() - p.last_signal_us) / 1000u;
        session->signal_age_ms = age < UINT32_MAX ? (uint32_t)age : UINT32_MAX;
    }
}

void audio_pipeline_get_source(audio_pipeline_source_t *source) {
    audio_source_t src = p.src == SRC_USB ? AUDIO_SOURCE_USB : input_source(in.input);
    source->heard = p.playing ? src : AUDIO_SOURCE_NONE;
    if (p.src == SRC_USB) {
        source->owner = p.open ? AUDIO_SOURCE_USB : AUDIO_SOURCE_NONE;
    } else {
        source->owner = input_ready() && in.st.input == in.input ? src : AUDIO_SOURCE_NONE;
    }
    source->emphasis = p.playing && p.src == SRC_INPUT &&
                       (ring_out_flags & SPDIF_TX_FLAG_EMPHASIS) != 0;
    source->input = in.st;
}
