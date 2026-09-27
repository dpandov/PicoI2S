// USB Audio Class 2.0 function: control requests and stream state.
//
// All tud_* callbacks in the firmware live here. TinyUSB calls them from
// tud_task() in the main loop, so they never run concurrently with the API.

#include "usb/usb_audio.h"

#include <string.h>

#include "hardware/sync.h"
#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#include "tusb.h"
#include "usb/usb_descriptors.h"
#include "diag/diag.h"

//--------------------------------------------------------------------+
// Sample rates
//--------------------------------------------------------------------+

// Ascending, as UAC2 requires for RANGE subranges.
const uint32_t usb_audio_sample_rates[] = {32000, 44100, 48000, 88200, 96000};

#define N_RATES  TU_ARRAY_SIZE(usb_audio_sample_rates)

const uint8_t usb_audio_n_sample_rates = (uint8_t) N_RATES;

#define DEFAULT_SAMPLE_RATE  48000u

// The driver silently truncates replies to its control buffer.
typedef audio_control_range_4_n_t(N_RATES) rate_range_t;
TU_VERIFY_STATIC(sizeof(rate_range_t) <= CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ,
                 "clock RANGE reply does not fit the audio control buffer");

TU_VERIFY_STATIC(CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ <= UINT16_MAX,
                 "receive FIFO depth does not fit usb_audio_stats_t");

// usb_audio_read() rounds down with a mask, and a FIFO depth that is a frame
// multiple keeps frames aligned even after an overflow.
TU_VERIFY_STATIC((AUDIO_N_CHANNELS * AUDIO_16BIT_SUBSLOT) == 4 &&
                 (AUDIO_N_CHANNELS * AUDIO_24BIT_SUBSLOT) == 8,
                 "usb_audio_read() assumes 4- and 8-byte stereo frames");
TU_VERIFY_STATIC(CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ % 8 == 0,
                 "receive FIFO depth must be a multiple of the frame size");

//--------------------------------------------------------------------+
// State
//--------------------------------------------------------------------+

static struct {
    bool     mounted;         // as last reported to the main loop
    uint8_t  stream_alt;      // AS_ALT_* of the streaming interface
    uint32_t sample_rate;
    uint8_t  resolution;      // from the active or last alt setting
    uint8_t  subslot_bytes;
    volatile uint32_t events; // pending USB_AUDIO_EVT_* bits
    bool     rx_misaligned;   // last packet was not whole frames
    usb_audio_stats_t stats;
} s;

static inline void raise_event(uint32_t evt)
{
    s.events |= evt;
}

// The audio driver drops the endpoints without calling the close callback,
// so a lost configuration also ends the stream.
static void note_detached(void)
{
    if (s.stream_alt != AS_ALT_ZERO_BW) {
        s.stream_alt = AS_ALT_ZERO_BW;
        raise_event(USB_AUDIO_EVT_STREAM_STOP);
    }
    if (s.mounted) {
        s.mounted = false;
        raise_event(USB_AUDIO_EVT_UNMOUNTED);
    }
}

// TinyUSB does not call tud_umount_cb() on a bus reset; detect it here.
static void reconcile_mount(void)
{
    if (s.mounted && !tud_mounted()) {
        note_detached();
    }
}

static bool rate_supported(uint32_t rate)
{
    for (uint32_t i = 0; i < N_RATES; i++) {
        if (usb_audio_sample_rates[i] == rate) {
            return true;
        }
    }
    return false;
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

void usb_audio_init(void)
{
    memset(&s, 0, sizeof(s));
    s.sample_rate   = DEFAULT_SAMPLE_RATE;
    s.resolution    = AUDIO_16BIT_RESOLUTION;
    s.subslot_bytes = AUDIO_16BIT_SUBSLOT;
    s.stats.fifo_depth = (uint16_t) CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;
}

usb_audio_link_t usb_audio_link_state(void)
{
    reconcile_mount();
    if (!s.mounted) {
        return USB_AUDIO_DETACHED;
    }
    return (s.stream_alt == AS_ALT_ZERO_BW) ? USB_AUDIO_IDLE
                                            : USB_AUDIO_STREAMING;
}

usb_audio_format_t usb_audio_format(void)
{
    usb_audio_format_t f = {
        .sample_rate   = s.sample_rate,
        .resolution    = s.resolution,
        .subslot_bytes = s.subslot_bytes,
    };
    return f;
}

uint32_t usb_audio_take_events(void)
{
    reconcile_mount();
    uint32_t irq = save_and_disable_interrupts();
    uint32_t evt = s.events;
    s.events = 0;
    restore_interrupts(irq);
    return evt;
}

void usb_audio_get_stats(usb_audio_stats_t *stats)
{
    *stats = s.stats;
    stats->fifo_level = tud_audio_available();
    stats->fifo_depth = (uint16_t) CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;
}

void usb_audio_fifo_max_reset(void)
{
    s.stats.fifo_max = 0;
}

uint32_t usb_audio_read(void *dst, uint32_t max_bytes)
{
    // The driver clears the FIFO when the stream closes.
    if (s.stream_alt == AS_ALT_ZERO_BW) {
        return 0;
    }

    uint32_t const frame = (uint32_t) AUDIO_N_CHANNELS * s.subslot_bytes;

    uint32_t n = tud_audio_available();
    if (n > max_bytes) {
        n = max_bytes;
    }
    if (n > UINT16_MAX) {          // tud_audio_read() takes a uint16_t
        n = UINT16_MAX;
    }
    n &= ~(frame - 1u);            // whole stereo frames only
    if (n == 0) {
        return 0;
    }

    uint32_t got = tud_audio_read(dst, (uint16_t) n);
    s.stats.rx_bytes += got;
    return got;
}

//--------------------------------------------------------------------+
// Device state callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void)
{
    // A bus reset plus re-enumeration can happen between main-loop polls.
    if (s.stream_alt != AS_ALT_ZERO_BW) {
        s.stream_alt = AS_ALT_ZERO_BW;
        raise_event(USB_AUDIO_EVT_STREAM_STOP);
    }
    s.mounted = true;
    raise_event(USB_AUDIO_EVT_MOUNTED);
}

void tud_umount_cb(void)
{
    note_detached();
}

//--------------------------------------------------------------------+
// Clock source requests
//--------------------------------------------------------------------+

static bool clock_get_request(uint8_t rhport, audio_control_request_t const *req)
{
    tusb_control_request_t const *p_req = (tusb_control_request_t const *) req;

    // Clock source controls are not per channel (UAC2 5.2.5.1).
    if (req->bChannelNumber != 0) {
        return false;
    }

    if (req->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (req->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_4_t cur = {
                .bCur = (int32_t) tu_htole32(s.sample_rate),
            };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &cur, sizeof(cur));
        }
        if (req->bRequest == AUDIO_CS_REQ_RANGE) {
            // Hosts may first ask for 2 bytes (wNumSubRanges); the control
            // layer truncates the full reply to wLength.
            rate_range_t range;
            range.wNumSubRanges = tu_htole16((uint16_t) N_RATES);
            for (uint32_t i = 0; i < N_RATES; i++) {
                range.subrange[i].bMin = (int32_t) tu_htole32(usb_audio_sample_rates[i]);
                range.subrange[i].bMax = (int32_t) tu_htole32(usb_audio_sample_rates[i]);
                range.subrange[i].bRes = 0;
            }
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &range, sizeof(range));
        }
        return false;
    }

    if (req->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
        req->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t valid = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &valid, sizeof(valid));
    }

    return false;
}

static bool clock_set_request(audio_control_request_t const *req, uint8_t const *buf)
{
    if (req->bChannelNumber != 0 ||
        req->bControlSelector != AUDIO_CS_CTRL_SAM_FREQ ||
        req->bRequest != AUDIO_CS_REQ_CUR ||
        req->wLength != sizeof(audio_control_cur_4_t)) {
        return false;
    }

    uint32_t const rate = tu_le32toh(tu_unaligned_read32(buf));

    if (!rate_supported(rate)) {
        s.stats.rate_rejects++;
        return false;
    }

    // Hosts re-send the current rate before every stream start.
    // TinyUSB only reads the feedback parameters at SET_INTERFACE, so a rate
    // change while streaming keeps the old feedback clamp until the stream
    // restarts. Windows, Linux and Apple change the rate with the stream closed.
    if (rate != s.sample_rate) {
        s.sample_rate = rate;
        s.stats.rate_changes++;
        raise_event(USB_AUDIO_EVT_RATE_CHANGED);
    }
    return true;
}

//--------------------------------------------------------------------+
// Audio class callbacks
//--------------------------------------------------------------------+

// Only the clock source has controls; everything else is STALLed.
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *req = (audio_control_request_t const *) p_request;
    diag_mark(DIAG_MARK_CLOCK_REQ);
    diag_cnt.clock_req++;

    if (req->bEntityID == UAC2_ENTITY_CLOCK) {
        return clock_get_request(rhport, req);
    }
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    (void) rhport;
    audio_control_request_t const *req = (audio_control_request_t const *) p_request;
    diag_mark(DIAG_MARK_CLOCK_REQ);
    diag_cnt.clock_req++;

    if (req->bEntityID == UAC2_ENTITY_CLOCK) {
        return clock_set_request(req, buf);
    }
    return false;
}

// Workaround for TinyUSB 0.18 on RP2040/RP2350: usbd_edpt_close() is a no-op
// for ISO endpoints, so a closed stream leaves the OUT endpoint armed and the
// next stream open panics ("ep 01 was already available"). Abort and clear
// both streaming endpoints in hardware when the stream closes.
static void iso_endpoints_disarm(void)
{
    uint8_t const ep_num = EPNUM_AUDIO_OUT & 0x0Fu;
    TU_VERIFY_STATIC((EPNUM_AUDIO_OUT & 0x0Fu) == 1 && (EPNUM_AUDIO_FB & 0x0Fu) == 1,
                     "abort bits below assume both streaming endpoints are EP1");
    uint32_t const mask = USB_EP_ABORT_EP1_OUT_BITS | USB_EP_ABORT_EP1_IN_BITS;

    hw_set_bits(&usb_hw->abort, mask);
    while ((usb_hw->abort_done & mask) != mask) {
        tight_loop_contents();
    }
    usb_dpram->ep_buf_ctrl[ep_num].out = 0;
    usb_dpram->ep_buf_ctrl[ep_num].in  = 0;
    usb_hw->abort_done = mask;                  // write-1-to-clear
    hw_clear_bits(&usb_hw->abort, mask);
}

// Called whenever an open stream closes, including a direct switch between
// the 16- and 24-bit alt settings (tud_audio_set_itf_cb() then follows).
bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    diag_mark(DIAG_MARK_CLOSE_EP);

    if (itf == ITF_NUM_AUDIO_STREAMING) {
        iso_endpoints_disarm();
    }

    if (itf == ITF_NUM_AUDIO_STREAMING && s.stream_alt != AS_ALT_ZERO_BW) {
        s.stream_alt = AS_ALT_ZERO_BW;
        raise_event(USB_AUDIO_EVT_STREAM_STOP);
    }
    return true;
}

// Called after the new alt setting's endpoints are open and before
// tud_audio_feedback_params_cb().
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
    diag_mark(DIAG_MARK_SET_ITF);
    diag_cnt.set_itf++;

    if (itf != ITF_NUM_AUDIO_STREAMING) {
        return true;
    }

    switch (alt) {
    case AS_ALT_ZERO_BW:
        s.stream_alt = AS_ALT_ZERO_BW;
        return true;

    case AS_ALT_16BIT:
        s.resolution    = AUDIO_16BIT_RESOLUTION;
        s.subslot_bytes = AUDIO_16BIT_SUBSLOT;
        break;

    case AS_ALT_24BIT:
        s.resolution    = AUDIO_24BIT_RESOLUTION;
        s.subslot_bytes = AUDIO_24BIT_SUBSLOT;
        break;

    default:
        return false;
    }

    s.stream_alt = alt;
    raise_event(USB_AUDIO_EVT_STREAM_START);
    diag_mark(DIAG_MARK_SET_ITF_DONE);
    return true;
}

// Receive path. On RP2350 the driver copies each packet from a linear buffer
// into the FIFO in tud_task(), so these run in the main loop, between reads.
// The FIFO is overwritable: a packet that does not fit overwrites the oldest
// data, and the next read skips to the newest FIFO-depth bytes. Packets and
// reads are whole frames and the depth is a frame multiple, so frames stay
// aligned; the lost audio is counted as an overrun.

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id,
                                   uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) func_id;
    (void) ep_out;
    (void) cur_alt_setting;

    uint32_t const frame = (uint32_t) AUDIO_N_CHANNELS * s.subslot_bytes;
    // Includes zero-length packets, after which TinyUSB 0.18 does not re-arm
    // the endpoint (the stream then stalls).
    if (n_bytes_received == 0 || n_bytes_received % frame != 0) {
        s.stats.bad_packets++;
        s.rx_misaligned = n_bytes_received != 0;
    }
    // tud_audio_available() saturates at the depth, so a FIFO that is still
    // overflowed also counts.
    if ((uint32_t) tud_audio_available() + n_bytes_received > CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ) {
        s.stats.overruns++;
    }
    return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id,
                                    uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) n_bytes_received;
    (void) func_id;
    (void) ep_out;
    (void) cur_alt_setting;

    if (s.rx_misaligned) {
        // Frame boundaries are unknown now: start over from the next packet.
        s.rx_misaligned = false;
        tud_audio_clear_ep_out_ff();
    }
    uint16_t const level = tud_audio_available();
    if (level > s.stats.fifo_max) {
        s.stats.fifo_max = level;
    }
    return true;
}

// FIFO_COUNT steers the host so the receive FIFO stays half full.
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param)
{
    (void) func_id;
    (void) alt_itf;

    diag_mark(DIAG_MARK_FB_PARAMS);
    memset(feedback_param, 0, sizeof(*feedback_param));
    feedback_param->method      = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = s.sample_rate;
}

// Full Speed feedback: 10.14 in 3 bytes for Apple hosts, 16.16 in 4 bytes
// for everyone else (see AUDIO_FB_EP_SIZE).
bool tud_audio_feedback_format_correction_cb(uint8_t func_id)
{
    (void) func_id;
    return usb_host_is_apple();
}
