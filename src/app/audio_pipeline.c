#include "app/audio_pipeline.h"

#include <stdio.h>

#include "pico.h"
#include "pico/time.h"

#include "clock/audio_clock.h"
#include "diag/diag.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"

// Ring of stereo frames: the main loop produces, the S/PDIF DMA interrupt
// consumes. Indices are free-running frame counts.
#define RING_FRAMES   512u
#define RING_MASK     (RING_FRAMES - 1u)
// Topped up to this level only, so most of the buffering stays in the USB
// FIFO, where the feedback endpoint regulates it.
#define RING_TARGET   (4u * SPDIF_TX_BUFFER_FRAMES)
#define READ_FRAMES   SPDIF_TX_BUFFER_FRAMES
#define IDLE_STOP_US  3000000u

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
    uint64_t last_audio_us;   // stream samples last moved into the ring
    uint32_t underruns;
    uint32_t start_failures;
} pipeline_t;

static pipeline_t p;

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

// Moves stream frames into the ring, up to RING_TARGET. Returns the count.
static uint32_t pump(const usb_audio_format_t *f) {
    uint32_t frame_bytes = 2u * f->subslot_bytes;
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
        __compiler_memory_barrier();
        ring_wr = wr + n;
        moved += n;
    }
    return moved;
}

static void top_up(const usb_audio_format_t *f, uint64_t now) {
    if (pump(f) > 0) {
        p.last_audio_us = now;
    }
}

static void output_stop(void) {
    spdif_tx_stop();
    ring_flush();
    p.playing = false;
    printf("spdif: stop\n");
}

// Stream opened, or its format changed while open. Event bits carry no
// order, so this is derived from the state.
static void stream_started(const usb_audio_format_t *f, uint64_t now) {
    p.rate = f->sample_rate;
    p.bits = f->resolution;
    p.playing = false;
    p.hold = false;
    // An opening stream counts as audio, so a running output is not stopped
    // during its pre-roll.
    p.last_audio_us = now;
    if (!spdif_tx_running()) {
        return;
    }
    if (spdif_tx_sample_rate() != f->sample_rate) {
        output_stop();
    } else if (spdif_tx_word_bits() != f->resolution) {
        spdif_tx_set_word_bits(f->resolution);
    }
}

// Pre-roll done: fill the ring and start the output if it is stopped.
static void begin_playing(const usb_audio_format_t *f, uint64_t now) {
    bool start = !spdif_tx_running();
    if (start) {
        audio_family_t fam = audio_clock_family(f->sample_rate);
        if (fam != audio_clock_current_family()) {
            audio_clock_set_family(fam);
            diag_clock_changed();
            printf("clock: %s family\n", fam == AUDIO_FAMILY_44K1 ? "44.1k" : "48k");
        }
    }
    top_up(f, now);
    if (start) {
        if (!spdif_tx_start(f->sample_rate, f->resolution, ring_source)) {
            p.start_failures++;
            p.hold = true;
            ring_flush();
            printf("spdif: start %lu failed (%lu)\n", (unsigned long)f->sample_rate,
                   (unsigned long)p.start_failures);
            return;
        }
        top_up(f, now);   // the start pre-filled two buffers from the ring
        printf("spdif: %lu Hz %u-bit\n", (unsigned long)f->sample_rate, f->resolution);
    }
    // After the fill: a flag raised before it is stale.
    ring_short = false;
    p.playing = true;
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

    if (open && (!p.open || f.sample_rate != p.rate || f.resolution != p.bits)) {
        stream_started(&f, now);
    }
    p.open = open;
    if (!open) {
        // The ring drains, then the output sends silence.
        p.playing = false;
    }

    if (ring_short) {
        ring_short = false;
        if (p.playing) {
            p.underruns++;
            p.playing = false;
        }
    }

    if (open && !p.playing && !p.hold) {
        usb_audio_stats_t st;
        usb_audio_get_stats(&st);
        if (st.fifo_level >= st.fifo_depth / 2u) {
            begin_playing(&f, now);
        }
    }

    if (p.playing) {
        top_up(&f, now);
    }

    if (spdif_tx_running() && now - p.last_audio_us >= IDLE_STOP_US) {
        output_stop();
    }
}

bool audio_pipeline_playing(void) {
    return p.playing;
}

uint32_t audio_pipeline_underruns(void) {
    return p.underruns;
}
