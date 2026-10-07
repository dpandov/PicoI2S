// USB audio stream or S/PDIF input -> S/PDIF output.
//
// The main loop moves PCM from the USB receive FIFO into a ring of 32-bit
// left-justified stereo frames; the S/PDIF DMA interrupt drains the ring at
// the transmitter's rate. The receive FIFO is held at its set point
// (usb_audio_fifo_setpoint()): in UAC2 mode by the USB feedback, in UAC1
// mode (adaptive) by trimming the transmitter rate to the host's
// (app/clock_servo.h).
//
// The output starts when a stream has pre-filled, and stops (pin idle) after
// 3 s without audio samples: stream closed, or open with nothing arriving.
// Digital silence inside an open stream counts as audio.
//
// The host's volume (usb_audio_volume()) is applied on the way into the
// ring, ramped 1 dB per chunk read: at 0 dB the samples pass unchanged;
// below it they are scaled, rounded to 24 bits and sent as 24-bit words
// whatever the stream depth. Muted streams send zeros and still count as
// audio.
//
// S/PDIF input (spdif/spdif_rx.h): USB has priority. After 3 s without an
// audible USB sample (not mounted, no stream, a stalled stream, or digital
// silence after the volume), a locked input with a known channel status
// takes the output; the first audible USB sample takes it back. Each switch
// stops the output and pre-rolls the new source. While the input plays,
// an open USB stream is drained to its set point and discarded. The input
// goes into the ring unchanged (0 dB), its emphasis flag carried per block
// to the transmitter, and the clock servo steers the output to the
// receiver's FIFO level in both USB modes. A discontinuity (unlock, input or
// rate change) is a stall: the ring drains and the 3-s rule applies; after
// it, USB owns the output again. Non-PCM input arrives as zeros and counts
// as audio, like a muted stream.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "spdif/spdif_rx.h"

typedef enum {
    AUDIO_SOURCE_NONE = 0,
    AUDIO_SOURCE_USB,
    AUDIO_SOURCE_COAX,
    AUDIO_SOURCE_TOSLINK,
} audio_source_t;

typedef struct {
    // Since the current or last stream started (USB stream opened or changed
    // format, or a switch to the other source).
    uint32_t underruns;       // ring ran dry while playing
    uint32_t overruns;        // USB packets that overwrote unread FIFO data
    uint32_t late_irqs;       // S/PDIF interrupts too late to refill in time
    uint32_t ring_min;        // lowest ring level while playing, in frames
    // Since the stream first started playing (excludes the start-up).
    uint32_t fifo_max;        // highest receive FIFO level, in bytes
    uint32_t loop_max_us;     // longest main-loop iteration
    // Since boot.
    uint32_t underruns_total;
    uint32_t overruns_total;
    // Now.
    int32_t  trim_ppb;        // transmitter rate trim (0 in UAC2 mode with USB)
    bool     clock_locked;    // UAC1 or input: playing and the clock servo locked
} audio_pipeline_stats_t;

// Session information for the host monitor (tools/monitor.py), all since boot.
typedef struct {
    uint32_t streams;         // USB stream starts (open, or format change while open)
    uint32_t frames;          // frames moved into the ring (wraps)
    uint32_t signal_age_ms;   // time since the last non-zero sample went into
                              // the ring (after the volume), UINT32_MAX if none
                              // yet: digital silence when large
} audio_pipeline_session_t;

typedef struct {
    audio_source_t heard;     // source whose samples are going out, NONE in a
                              // pre-roll, underrun, stall or with the output off
    audio_source_t owner;     // source the output belongs to while it stays
                              // connected (USB stream open, input locked)
    bool emphasis;            // the input samples going out are pre-emphasised
    spdif_rx_status_t input;  // receiver status at the last audio_pipeline_task()
} audio_pipeline_source_t;

// Call after spdif_tx_init() and spdif_rx_init().
void audio_pipeline_init(void);

// Call every main-loop iteration, after tud_task() and spdif_rx_task().
// Also applies the transmitter rate trim (spdif_tx_trim_task()).
void audio_pipeline_task(void);

// USB stream samples are going out (pre-roll done, no underrun since).
bool audio_pipeline_playing(void);

void audio_pipeline_get_stats(audio_pipeline_stats_t *stats);
void audio_pipeline_get_session(audio_pipeline_session_t *session);
void audio_pipeline_get_source(audio_pipeline_source_t *source);
