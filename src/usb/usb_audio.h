// USB Audio Class 2.0 function: received PCM stream and its format.
// Main loop only; not callable from interrupts.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    USB_AUDIO_DETACHED = 0,   // not configured by a host
    USB_AUDIO_IDLE,           // configured, streaming interface at alt 0
    USB_AUDIO_STREAMING,      // streaming interface at alt 1 or 2
} usb_audio_link_t;

typedef struct {
    uint32_t sample_rate;     // Hz, one of the supported rates
    uint8_t  resolution;      // valid bits per sample: 16 or 24
    uint8_t  subslot_bytes;   // bytes per sample in the stream: 2 or 4
} usb_audio_format_t;

typedef struct {
    uint32_t rx_bytes;        // total payload bytes read via usb_audio_read()
    uint32_t rate_changes;    // accepted SET_CUR requests that changed the rate
    uint32_t rate_rejects;    // SET_CUR requests with an unsupported rate
    uint16_t fifo_level;      // bytes currently in the receive FIFO
    uint16_t fifo_depth;      // receive FIFO capacity in bytes
} usb_audio_stats_t;

// Event bits returned by usb_audio_take_events().
#define USB_AUDIO_EVT_MOUNTED         (1u << 0)
#define USB_AUDIO_EVT_UNMOUNTED       (1u << 1)
#define USB_AUDIO_EVT_STREAM_START    (1u << 2)
#define USB_AUDIO_EVT_STREAM_STOP     (1u << 3)
#define USB_AUDIO_EVT_RATE_CHANGED    (1u << 4)

// Supported sample rates, ascending.
extern const uint32_t usb_audio_sample_rates[];
extern const uint8_t  usb_audio_n_sample_rates;

// Call once before tusb_init().
void usb_audio_init(void);

usb_audio_link_t   usb_audio_link_state(void);

// Last format set by the host (48 kHz / 16-bit at boot); valid when idle.
usb_audio_format_t usb_audio_format(void);

// Returns and clears pending USB_AUDIO_EVT_* bits. Bits do not encode order
// (e.g. STOP and START on a direct 16/24-bit switch); read the resulting
// state from usb_audio_link_state() and usb_audio_format().
uint32_t usb_audio_take_events(void);

void usb_audio_get_stats(usb_audio_stats_t *stats);

// Reads up to max_bytes of interleaved little-endian PCM, whole stereo
// frames only. Returns the number of bytes read.
uint32_t usb_audio_read(void *dst, uint32_t max_bytes);
