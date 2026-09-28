// USB audio function (UAC2 or UAC1): received PCM stream and its format.
// Main loop only; not callable from interrupts.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Selected once at boot (UAC_MODE_PIN).
typedef enum {
    // Asynchronous with a feedback endpoint: 32 to 96 kHz, 16 and 24 bit.
    USB_AUDIO_MODE_UAC2 = 0,
    // PCM2704-like for TVs and old hosts: adaptive endpoint, no feedback,
    // 32 / 44.1 / 48 kHz, 16 bit. The output clock follows the host
    // (app/clock_servo.h).
    USB_AUDIO_MODE_UAC1,
} usb_audio_mode_t;

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

// Feature Unit volume range: 0 dB down to -USB_AUDIO_ATT_MAX_DB in 1 dB
// steps.
#define USB_AUDIO_ATT_MAX_DB  64u

// Master volume set by the host. 0 dB, unmuted at boot and on every
// enumeration.
typedef struct {
    bool    mute;             // output zeros: mute control, or a volume below
                              // the range
    uint8_t att_db;           // attenuation, 0 .. USB_AUDIO_ATT_MAX_DB
} usb_audio_volume_t;

// Counters are cumulative since boot.
typedef struct {
    uint32_t rx_bytes;        // total payload bytes read via usb_audio_read()
    uint32_t rate_changes;    // accepted SET_CUR requests that changed the rate
    uint32_t rate_rejects;    // SET_CUR requests with an unsupported rate
    uint32_t overruns;        // packets that overwrote unread FIFO data
    uint32_t bad_packets;     // zero-length packets, or not whole stereo frames
    uint16_t fifo_level;      // bytes currently in the receive FIFO
    uint16_t fifo_depth;      // receive FIFO capacity in bytes
    uint16_t fifo_max;        // highest level after a packet since the last
                              // usb_audio_fifo_max_reset()
} usb_audio_stats_t;

// Event bits returned by usb_audio_take_events().
#define USB_AUDIO_EVT_MOUNTED         (1u << 0)
#define USB_AUDIO_EVT_UNMOUNTED       (1u << 1)
#define USB_AUDIO_EVT_STREAM_START    (1u << 2)
#define USB_AUDIO_EVT_STREAM_STOP     (1u << 3)
#define USB_AUDIO_EVT_RATE_CHANGED    (1u << 4)

// Supported sample rates in UAC2 mode, ascending. UAC1 offers those up to
// UAC1_MAX_SAMPLE_RATE (usb_descriptors.h).
extern const uint32_t usb_audio_sample_rates[];
extern const uint8_t  usb_audio_n_sample_rates;

// Call once before tusb_init().
void usb_audio_init(usb_audio_mode_t mode);

// Fixed from usb_audio_init() on; the one call that is safe in interrupts.
usb_audio_mode_t   usb_audio_mode(void);

usb_audio_link_t   usb_audio_link_state(void);

// Last format set by the host (48 kHz / 16-bit at boot); valid when idle.
// UAC1 hosts may set the rate only after opening the stream, and some never
// do: 48 kHz is assumed then (reset on every enumeration).
usb_audio_format_t usb_audio_format(void);

// Returns and clears pending USB_AUDIO_EVT_* bits. Bits do not encode order
// (e.g. STOP and START on a direct 16/24-bit switch); read the resulting
// state from usb_audio_link_state() and usb_audio_format().
uint32_t usb_audio_take_events(void);

// Changes as the host's Feature Unit requests arrive in tud_task(), so it
// is stable between two tud_task() calls.
usb_audio_volume_t usb_audio_volume(void);

void usb_audio_get_stats(usb_audio_stats_t *stats);
void usb_audio_fifo_max_reset(void);

// Bytes currently in the receive FIFO.
uint32_t usb_audio_fifo_level(void);

// Receive FIFO level held while streaming, in 1 ms packets of the current
// format: the same time at every format (the FIFO itself is sized in bytes
// for 96 kHz / 24-bit, where 6 packets is half of it). It is the latency
// through the FIFO and the margin against a late or early host.
#define FIFO_SETPOINT_PACKETS 6u
uint32_t usb_audio_fifo_setpoint(uint32_t sample_rate, uint8_t subslot_bytes);

// Reads up to max_bytes of interleaved little-endian PCM, whole stereo
// frames only. Returns the number of bytes read.
uint32_t usb_audio_read(void *dst, uint32_t max_bytes);
