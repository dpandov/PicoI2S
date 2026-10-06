#include "app/audio_pipeline.h"

#include "pico.h"
#include "pico/time.h"

#include "app/clock_servo.h"
#include "app/log.h"
#include "clock/audio_clock.h"
#include "diag/diag.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"

// Ring of stereo frames: the main loop produces, the S/PDIF DMA interrupt
// consumes. Indices are free-running frame counts.
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
// Ramped attenuation value for silence (mute): one step below -64 dB.
#define ATT_SILENT    (USB_AUDIO_ATT_MAX_DB + 1u)

_Static_assert((RING_FRAMES & RING_MASK) == 0, "ring size must be a power of two");
_Static_assert(RING_TARGET <= RING_FRAMES, "ring target exceeds the ring");

static int32_t ring[RING_FRAMES * 2];
static volatile uint32_t ring_wr;      // written by the main loop only
static volatile uint32_t ring_rd;      // written by the interrupt only
static volatile bool ring_short;       // the interrupt found too few frames

static union {
    int16_t s16[READ_FRAMES * 2];
    int32_t s32[READ_FRAMES * 2];
} rx;

typedef struct {
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
    uint32_t start_failures;
    uint32_t overruns_base;   // counts when the stream started
    uint32_t late_irqs_base;
    audio_pipeline_stats_t st;
} pipeline_t;

static pipeline_t p;

// UAC1: steers the output rate to the host's. Runs while playing.
static clock_servo_t servo;

static bool adaptive(void) {
    return usb_audio_mode() == USB_AUDIO_MODE_UAC1;
}

// The interrupt runs to completion before the main loop resumes, so only
// the producer side needs ordering. A short ring plays what it has, then
// silence.
static void __not_in_flash_func(ring_source)(int32_t *pcm, uint32_t n_frames) {
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
}

// Only while the transmitter is stopped (the interrupt owns ring_rd).
static void ring_flush(void) {
    ring_rd = ring_wr;
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

// Moves the applied attenuation 1 dB towards the target, once per chunk
// (96 frames when steady, 2 ms at 48 kHz), so a jump or a mute fades as on
// the PCM2704.
static void att_step(uint8_t target) {
    if (p.att < target) {
        p.att++;
    } else if (p.att > target) {
        p.att--;
    }
}

// Attenuates or mutes n frames just written at ring index wr.
static void apply_volume(uint32_t wr, uint32_t n, uint8_t att) {
    int32_t gain = att == ATT_SILENT ? 0 : gain_q31[att];
    for (uint32_t i = 0; i < n; i++) {
        uint32_t k = ((wr + i) & RING_MASK) * 2u;
        ring[k]      = scale_sample(ring[k], gain);
        ring[k + 1u] = scale_sample(ring[k + 1u], gain);
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
        att_step(target);
        if (p.att != 0) {
            apply_volume(wr, n, p.att);
        }
        // What goes out on the line: a muted stream is digital silence.
        if (chunk_has_signal(wr, n)) {
            p.last_signal_us = now;
            p.had_signal = true;
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
// order, so this is derived from the state.
static void stream_started(const usb_audio_format_t *f, uint64_t now) {
    p.rate = f->sample_rate;
    p.bits = f->resolution;
    p.playing = false;
    p.hold = false;
    p.streams++;
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
static void servo_start(const usb_audio_format_t *f, bool fresh) {
    clock_servo_params_t sp = {
        .sample_rate = f->sample_rate,
        .frame_bytes = 2u * f->subslot_bytes,
        .setpoint_bytes = fifo_setpoint(f),
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

// Pre-roll done: fill the ring and start the output if it is stopped.
static void begin_playing(const usb_audio_format_t *f, uint64_t now) {
    bool start = !spdif_tx_running();
    // Starts at the requested volume without a ramp: silence precedes it
    // (pre-roll, underrun or stopped output).
    p.att = att_target(usb_audio_volume());
    if (start && audio_clock_rate_changes(f->sample_rate)) {
        uint32_t hz = audio_clock_set_rate(f->sample_rate);
        diag_clock_changed();
        log_printf("clock: %lu Hz\n", (unsigned long)hz);
    }
    top_up(f, now);
    if (start) {
        uint8_t bits = output_word_bits(f->resolution);
        if (!spdif_tx_start(f->sample_rate, bits, ring_source)) {
            p.start_failures++;
            p.hold = true;
            ring_flush();
            log_printf("spdif: start %lu failed (%lu)\n", (unsigned long)f->sample_rate,
                       (unsigned long)p.start_failures);
            return;
        }
        top_up(f, now);   // the start pre-filled two buffers from the ring
        log_printf("spdif: %lu Hz %u-bit\n", (unsigned long)f->sample_rate, bits);
    }
    if (adaptive()) {
        servo_start(f, start);
    }
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

static void log_stream_end(void) {
    audio_pipeline_stats_t st;
    audio_pipeline_get_stats(&st);
    log_printf("pipe: end ur%lu ov%lu lt%lu rmin%lu fmax%lu lp%lu tr%ld\n",
               (unsigned long)st.underruns, (unsigned long)st.overruns,
               (unsigned long)st.late_irqs, (unsigned long)st.ring_min,
               (unsigned long)st.fifo_max, (unsigned long)st.loop_max_us,
               (long)st.trim_ppb);
}

void audio_pipeline_init(void) {
    ring_wr = 0;
    ring_rd = 0;
    ring_short = false;
    p = (pipeline_t){0};
}

void audio_pipeline_task(void) {
    uint64_t now = time_us_64();
    bool open = usb_audio_link_state() == USB_AUDIO_STREAMING;
    usb_audio_format_t f = usb_audio_format();

    if (p.loop_reset) {
        p.loop_reset = false;
        diag_loop_max_reset();
    }

    bool changed = f.sample_rate != p.rate || f.resolution != p.bits;
    if (p.open && (!open || changed)) {
        log_stream_end();
    }
    if (open && (!p.open || changed)) {
        stream_started(&f, now);
    }
    p.open = open;
    if (!open) {
        // The ring drains, then the output sends silence.
        p.playing = false;
    }
    // Follows a change of the stream depth; volume changes are synced per
    // chunk in pump().
    sync_word_bits();

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

    if (open && !p.playing && !p.hold) {
        usb_audio_stats_t us;
        usb_audio_get_stats(&us);
        if (us.fifo_level >= preroll_bytes(&f, us.fifo_depth)) {
            begin_playing(&f, now);
        }
    }

    if (p.playing) {
        // Lowest point: only the interrupt has run since the last top-up.
        uint32_t level = ring_wr - ring_rd;
        if (level < p.st.ring_min) {
            p.st.ring_min = level;
        }
        top_up(&f, now);
        // Only while playing. Otherwise (underrun, stream closed) the trim
        // stays, and the next begin_playing() resumes the servo.
        if (adaptive() && clock_servo_update(&servo, usb_audio_fifo_level(), now)) {
            spdif_tx_set_trim_ppb(clock_servo_trim_ppb(&servo));
        }
    }

    if (spdif_tx_running() && now - p.last_audio_us >= IDLE_STOP_US) {
        output_stop();
    }

    spdif_tx_trim_task();
}

bool audio_pipeline_playing(void) {
    return p.playing;
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
    stats->clock_locked = p.playing && adaptive() && clock_servo_locked(&servo);
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
