// USB audio function (UAC2 or UAC1, see usb_audio_mode_t): control
// requests and stream state.
//
// All tud_* callbacks in the firmware live here. Control and state callbacks
// run from tud_task() in the main loop, so they never run concurrently with
// the API. The receive and feedback paths run in the USB interrupt:
// tud_audio_rx_done_isr() and __wrap_usbd_edpt_xfer().

#include "usb/usb_audio.h"

#include <string.h>

#include "hardware/sync.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "usb/usb_descriptors.h"
#include "diag/diag.h"

//--------------------------------------------------------------------+
// Sample rates
//--------------------------------------------------------------------+

// Ascending, as UAC2 requires for RANGE subranges. UAC1 offers the first
// three (usb_descriptors.c).
const uint32_t usb_audio_sample_rates[] = {32000, 44100, 48000, 88200, 96000};

#define N_RATES  TU_ARRAY_SIZE(usb_audio_sample_rates)

const uint8_t usb_audio_n_sample_rates = (uint8_t) N_RATES;

#define DEFAULT_SAMPLE_RATE  48000u

// The driver STALLs replies that do not fit its control buffer.
typedef audio20_control_range_4_n_t(N_RATES) rate_range_t;
TU_VERIFY_STATIC(sizeof(rate_range_t) <= CFG_TUD_AUDIO_CTRL_BUF_SZ,
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
    usb_audio_mode_t mode;    // fixed from usb_audio_init() on
    bool     mounted;         // as last reported to the main loop
    uint8_t  stream_alt;      // AS_ALT_* of the streaming interface
    uint32_t sample_rate;
    uint8_t  resolution;      // from the active or last alt setting
    uint8_t  subslot_bytes;
    bool     mute;            // Feature Unit mute control
    bool     vol_silent;      // volume set below the range
    uint8_t  att_db;          // volume on the 1 dB grid, clamped to the range
    volatile uint32_t events; // pending USB_AUDIO_EVT_* bits
    volatile bool rx_misaligned; // set in the USB interrupt
    bool     rx_halted;       // halt of the current stream already counted
    usb_audio_stats_t stats;  // overruns, bad_packets and fifo_max are also
                              // written in the USB interrupt
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

// After a zero-length packet the driver returns before re-arming the OUT
// endpoint and without calling tud_audio_rx_done_isr(), so reception stops
// until the next SET_INTERFACE. The endpoint is then left idle: count that
// once per stream as a bad packet. Race-free in the main loop, because the
// interrupt clears and re-sets the busy flag within one packet.
static void check_rx_halt(void)
{
    if (s.stream_alt != AS_ALT_ZERO_BW && !s.rx_halted &&
        !usbd_edpt_busy(0, EPNUM_AUDIO_OUT)) {
        s.rx_halted = true;
        uint32_t irq = save_and_disable_interrupts();
        s.stats.bad_packets++;
        restore_interrupts(irq);
    }
}

static bool rate_supported(uint32_t rate)
{
    if (s.mode == USB_AUDIO_MODE_UAC1 && rate > UAC1_MAX_SAMPLE_RATE) {
        return false;
    }
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

// Accepts a supported rate from the host; raises RATE_CHANGED on a change.
static bool set_sample_rate(uint32_t rate)
{
    if (!rate_supported(rate)) {
        s.stats.rate_rejects++;
        return false;
    }
    if (rate != s.sample_rate) {
        s.sample_rate = rate;
        s.stats.rate_changes++;
        raise_event(USB_AUDIO_EVT_RATE_CHANGED);
    }
    return true;
}

static void volume_reset(void)
{
    s.mute       = false;
    s.vol_silent = false;
    s.att_db     = 0;
}

void usb_audio_init(usb_audio_mode_t mode)
{
    memset(&s, 0, sizeof(s));
    s.mode          = mode;
    s.sample_rate   = DEFAULT_SAMPLE_RATE;
    s.resolution    = AUDIO_16BIT_RESOLUTION;
    s.subslot_bytes = AUDIO_16BIT_SUBSLOT;
    s.stats.fifo_depth = (uint16_t) CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;
    volume_reset();
}

usb_audio_mode_t usb_audio_mode(void)
{
    return s.mode;
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
    check_rx_halt();
    uint32_t irq = save_and_disable_interrupts();
    uint32_t evt = s.events;
    s.events = 0;
    restore_interrupts(irq);
    return evt;
}

usb_audio_volume_t usb_audio_volume(void)
{
    usb_audio_volume_t v = {
        .mute   = s.mute || s.vol_silent,
        .att_db = s.att_db,
    };
    return v;
}

void usb_audio_get_stats(usb_audio_stats_t *stats)
{
    uint32_t irq = save_and_disable_interrupts();
    *stats = s.stats;
    restore_interrupts(irq);
    stats->fifo_level = tud_audio_available();
    stats->fifo_depth = (uint16_t) CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;
}

void usb_audio_fifo_max_reset(void)
{
    s.stats.fifo_max = 0;
}

uint32_t usb_audio_fifo_level(void)
{
    return tud_audio_available();
}

uint32_t usb_audio_read(void *dst, uint32_t max_bytes)
{
    // The driver clears the FIFO when the stream closes.
    if (s.stream_alt == AS_ALT_ZERO_BW) {
        return 0;
    }

    if (s.rx_misaligned) {
        // Frame boundaries are unknown now: start over from the next packet.
        uint32_t irq = save_and_disable_interrupts();
        s.rx_misaligned = false;
        tud_audio_clear_ep_out_ff();
        restore_interrupts(irq);
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
    // UAC1 hosts need not set the rate, so a new host starts from the
    // default, not from the rate the previous host left.
    if (s.mode == USB_AUDIO_MODE_UAC1 && s.sample_rate != DEFAULT_SAMPLE_RATE) {
        s.sample_rate = DEFAULT_SAMPLE_RATE;
        raise_event(USB_AUDIO_EVT_RATE_CHANGED);
    }
    // Hosts set their volume on connect; one that does not gets 0 dB.
    volume_reset();
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

static bool clock_get_request(uint8_t rhport, tusb_control_request_t const *p_req,
                              uint8_t selector)
{
    if (selector == AUDIO20_CS_CTRL_SAM_FREQ) {
        if (p_req->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_4_t cur = {
                .bCur = (int32_t) tu_htole32(s.sample_rate),
            };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &cur, sizeof(cur));
        }
        if (p_req->bRequest == AUDIO20_CS_REQ_RANGE) {
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

    if (selector == AUDIO20_CS_CTRL_CLK_VALID && p_req->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t valid = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &valid, sizeof(valid));
    }

    return false;
}

static bool clock_set_request(tusb_control_request_t const *p_req, uint8_t selector,
                              uint8_t const *buf)
{
    if (selector != AUDIO20_CS_CTRL_SAM_FREQ ||
        p_req->bRequest != AUDIO20_CS_REQ_CUR ||
        p_req->wLength != sizeof(audio20_control_cur_4_t)) {
        return false;
    }

    uint32_t const rate = tu_le32toh(tu_unaligned_read32(buf));

    // Hosts re-send the current rate before every stream start.
    // TinyUSB only reads the feedback parameters at SET_INTERFACE, so a rate
    // change while streaming keeps the old feedback clamp until the stream
    // restarts. Windows, Linux and Apple change the rate with the stream closed.
    return set_sample_rate(rate);
}

//--------------------------------------------------------------------+
// Feature Unit requests: master mute and volume
//--------------------------------------------------------------------+

// Same layout in UAC1 and UAC2: mute 1 byte, volume int16 in 1/256 dB.
// The request codes differ: UAC2 has CUR and RANGE, UAC1 has SET_CUR,
// GET_CUR, GET_MIN, GET_MAX and GET_RES. TinyUSB routes both to the entity
// callbacks. Selector codes are the same in both (mute 1, volume 2).
TU_VERIFY_STATIC((int) AUDIO10_FU_CTRL_MUTE == (int) AUDIO20_FU_CTRL_MUTE &&
                 (int) AUDIO10_FU_CTRL_VOLUME == (int) AUDIO20_FU_CTRL_VOLUME,
                 "UAC1 and UAC2 Feature Unit selectors differ");
TU_VERIFY_STATIC(UAC1_ENTITY_FEATURE_UNIT == UAC2_ENTITY_FEATURE_UNIT,
                 "Feature Unit IDs differ between the modes");

#define VOLUME_RES      256
#define VOLUME_MIN      (-(int32_t) USB_AUDIO_ATT_MAX_DB * VOLUME_RES)
#define VOLUME_MAX      0

typedef audio20_control_range_2_n_t(1) volume_range_t;
TU_VERIFY_STATIC(sizeof(volume_range_t) <= CFG_TUD_AUDIO_CTRL_BUF_SZ,
                 "volume RANGE reply does not fit the audio control buffer");

typedef enum {
    FU_ATTR_CUR,
    FU_ATTR_MIN,
    FU_ATTR_MAX,
    FU_ATTR_RES,
    FU_ATTR_RANGE,
    FU_ATTR_NONE,
} fu_attr_t;

static fu_attr_t fu_get_attr(uint8_t req)
{
    if (s.mode == USB_AUDIO_MODE_UAC2) {
        return req == AUDIO20_CS_REQ_CUR   ? FU_ATTR_CUR
             : req == AUDIO20_CS_REQ_RANGE ? FU_ATTR_RANGE
             : FU_ATTR_NONE;
    }
    switch (req) {
    case AUDIO10_CS_REQ_GET_CUR: return FU_ATTR_CUR;
    case AUDIO10_CS_REQ_GET_MIN: return FU_ATTR_MIN;
    case AUDIO10_CS_REQ_GET_MAX: return FU_ATTR_MAX;
    case AUDIO10_CS_REQ_GET_RES: return FU_ATTR_RES;
    default:                     return FU_ATTR_NONE;
    }
}

// Rounds to the 1 dB grid (half a step towards more attenuation). Below
// the range, including -infinity (0x8000), the output is silent and CUR
// reads back the minimum.
static void volume_set(int16_t v)
{
    if (v >= VOLUME_MAX) {
        s.att_db     = 0;
        s.vol_silent = false;
        return;
    }
    uint32_t const att = ((uint32_t) -(int32_t) v + VOLUME_RES / 2u) / VOLUME_RES;
    s.vol_silent = att > USB_AUDIO_ATT_MAX_DB;
    s.att_db     = (uint8_t) (s.vol_silent ? USB_AUDIO_ATT_MAX_DB : att);
}

static bool reply_i16(uint8_t rhport, tusb_control_request_t const *p_req, int32_t v)
{
    audio20_control_cur_2_t r = { .bCur = (int16_t) tu_htole16((uint16_t) (int16_t) v) };
    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &r, sizeof(r));
}

static bool fu_get_request(uint8_t rhport, tusb_control_request_t const *p_req, uint8_t selector)
{
    fu_attr_t const attr = fu_get_attr(p_req->bRequest);

    if (selector == AUDIO20_FU_CTRL_MUTE && attr == FU_ATTR_CUR) {
        audio20_control_cur_1_t cur = { .bCur = s.mute ? 1 : 0 };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &cur, sizeof(cur));
    }
    if (selector != AUDIO20_FU_CTRL_VOLUME) {
        return false;
    }
    switch (attr) {
    case FU_ATTR_CUR: return reply_i16(rhport, p_req, -(int32_t) s.att_db * VOLUME_RES);
    case FU_ATTR_MIN: return reply_i16(rhport, p_req, VOLUME_MIN);
    case FU_ATTR_MAX: return reply_i16(rhport, p_req, VOLUME_MAX);
    case FU_ATTR_RES: return reply_i16(rhport, p_req, VOLUME_RES);
    case FU_ATTR_RANGE: {
        volume_range_t range;
        range.wNumSubRanges    = tu_htole16(1);
        range.subrange[0].bMin =(int16_t) tu_htole16((uint16_t) (int16_t) VOLUME_MIN);
        range.subrange[0].bMax = (int16_t) tu_htole16((uint16_t) VOLUME_MAX);
        range.subrange[0].bRes = tu_htole16(VOLUME_RES);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_req, &range, sizeof(range));
    }
    default:
        return false;
    }
}

TU_VERIFY_STATIC((int) AUDIO10_CS_REQ_SET_CUR == (int) AUDIO20_CS_REQ_CUR, "SET CUR codes differ");

// UAC1 SET_CUR and UAC2 CUR share the code 0x01; only CUR is writable.
static bool fu_set_request(tusb_control_request_t const *p_req, uint8_t selector,
                           uint8_t const *buf)
{
    uint16_t const len = tu_le16toh(p_req->wLength);
    if (p_req->bRequest != AUDIO20_CS_REQ_CUR) {
        return false;
    }
    if (selector == AUDIO20_FU_CTRL_MUTE && len == 1) {
        s.mute = buf[0] != 0;
        return true;
    }
    if (selector == AUDIO20_FU_CTRL_VOLUME && len == 2) {
        volume_set((int16_t) tu_le16toh(tu_unaligned_read16(buf)));
        return true;
    }
    return false;
}

//--------------------------------------------------------------------+
// Audio class callbacks
//--------------------------------------------------------------------+

// Entities with controls: the Feature Unit (master channel only) and, in
// UAC2, the clock source (whose controls are not per channel, UAC2
// 5.2.5.1). Everything else is STALLed. Control requests run in tud_task(),
// so the volume changes only there (see usb_audio_volume()).
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint16_t const value   = tu_le16toh(p_request->wValue);
    uint8_t const  entity  = TU_U16_HIGH(tu_le16toh(p_request->wIndex));
    uint8_t const  channel = TU_U16_LOW(value);

    if (entity == UAC2_ENTITY_FEATURE_UNIT) {
        diag_mark(DIAG_MARK_VOLUME_REQ);
        return channel == 0 && fu_get_request(rhport, p_request, TU_U16_HIGH(value));
    }
    diag_mark(DIAG_MARK_CLOCK_REQ);
    diag_cnt.clock_req++;
    if (s.mode == USB_AUDIO_MODE_UAC2 && entity == UAC2_ENTITY_CLOCK && channel == 0) {
        return clock_get_request(rhport, p_request, TU_U16_HIGH(value));
    }
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    (void) rhport;
    uint16_t const value   = tu_le16toh(p_request->wValue);
    uint8_t const  entity  = TU_U16_HIGH(tu_le16toh(p_request->wIndex));
    uint8_t const  channel = TU_U16_LOW(value);

    if (entity == UAC2_ENTITY_FEATURE_UNIT) {
        diag_mark(DIAG_MARK_VOLUME_REQ);
        return channel == 0 && fu_set_request(p_request, TU_U16_HIGH(value), buf);
    }
    diag_mark(DIAG_MARK_CLOCK_REQ);
    diag_cnt.clock_req++;
    if (s.mode == USB_AUDIO_MODE_UAC2 && entity == UAC2_ENTITY_CLOCK && channel == 0) {
        return clock_set_request(p_request, TU_U16_HIGH(value), buf);
    }
    return false;
}

// UAC1: the data endpoint's sampling frequency control, 3 bytes. Hosts
// usually send it after selecting alt 1, so the rate can change on an open
// stream (the pipeline then restarts it); some repeat it, some never send it.
static bool uac1_rate_request(tusb_control_request_t const *p_request, uint8_t req)
{
    if (s.mode != USB_AUDIO_MODE_UAC1) {
        return false;       // UAC2 has no endpoint controls
    }
    diag_mark(DIAG_MARK_CLOCK_REQ);
    diag_cnt.clock_req++;
    // A GET reply is truncated to wLength by the control layer.
    return TU_U16_LOW(tu_le16toh(p_request->wIndex)) == EPNUM_AUDIO_OUT &&
           TU_U16_HIGH(tu_le16toh(p_request->wValue)) == AUDIO10_EP_CTRL_SAMPLING_FREQ &&
           p_request->bRequest == req &&
           (req == AUDIO10_CS_REQ_GET_CUR || tu_le16toh(p_request->wLength) == 3);
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    if (!uac1_rate_request(p_request, AUDIO10_CS_REQ_GET_CUR)) {
        return false;
    }
    uint8_t cur[3] = {
        (uint8_t) s.sample_rate, (uint8_t) (s.sample_rate >> 8), (uint8_t) (s.sample_rate >> 16),
    };
    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, cur, sizeof(cur));
}

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    (void) rhport;
    if (!uac1_rate_request(p_request, AUDIO10_CS_REQ_SET_CUR)) {
        return false;
    }
    return set_sample_rate((uint32_t) buf[0] | ((uint32_t) buf[1] << 8) | ((uint32_t) buf[2] << 16));
}

// Called whenever an open stream closes, including a direct switch between
// the 16- and 24-bit alt settings (tud_audio_set_itf_cb() then follows).
bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    diag_mark(DIAG_MARK_CLOSE_EP);

    if (itf == ITF_NUM_AUDIO_STREAMING && s.stream_alt != AS_ALT_ZERO_BW) {
        s.stream_alt = AS_ALT_ZERO_BW;
        raise_event(USB_AUDIO_EVT_STREAM_STOP);
    }
    return true;
}

// Called after the new alt setting's endpoints are open and armed, and
// before tud_audio_feedback_params_cb().
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
        if (s.mode == USB_AUDIO_MODE_UAC1) {
            return false;
        }
        s.resolution    = AUDIO_24BIT_RESOLUTION;
        s.subslot_bytes = AUDIO_24BIT_SUBSLOT;
        break;

    default:
        return false;
    }

    s.stream_alt = alt;
    s.rx_halted  = false;
    raise_event(USB_AUDIO_EVT_STREAM_START);
    diag_mark(DIAG_MARK_SET_ITF_DONE);
    return true;
}

// USB interrupt, after the driver copied the packet into the FIFO and
// re-armed the endpoint. The FIFO is overwritable: a packet that does not
// fit overwrites the oldest data, and the next read skips to the newest
// FIFO-depth bytes. Packets and reads are whole frames and the depth is a
// frame multiple, so frames stay aligned; the lost audio is counted as an
// overrun.
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id,
                           uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) func_id;
    (void) ep_out;

    uint32_t const subslot = (cur_alt_setting == AS_ALT_24BIT) ? AUDIO_24BIT_SUBSLOT
                                                               : AUDIO_16BIT_SUBSLOT;
    if (n_bytes_received % (AUDIO_N_CHANNELS * subslot) != 0) {
        s.stats.bad_packets++;
        s.rx_misaligned = true;
    }

    tu_fifo_t const *ff = tud_audio_get_ep_out_ff();
    uint16_t const used = tu_ff_overflow_count(ff->depth, ff->wr_idx, ff->rd_idx);
    if (used > ff->depth) {
        s.stats.overruns++;
    }
    uint16_t const level = tu_min16(used, ff->depth);
    if (level > s.stats.fifo_max) {
        s.stats.fifo_max = level;
    }
    return true;
}

// FIFO_COUNT steers the host so the receive FIFO stays half full. UAC2
// only: the UAC1 descriptors have no feedback endpoint.
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param)
{
    (void) func_id;
    (void) alt_itf;

    diag_mark(DIAG_MARK_FB_PARAMS);
    memset(feedback_param, 0, sizeof(*feedback_param));
    feedback_param->method      = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = s.sample_rate;
}

//--------------------------------------------------------------------+
// Apple feedback format
//--------------------------------------------------------------------+

// TinyUSB sends UAC2 feedback as 16.16 in 4 bytes, which Windows requires.
// Apple hosts require 10.14 in 3 bytes on Full Speed (see AUDIO_FB_EP_SIZE),
// so the feedback transfer is rewritten in place. TinyUSB's own calls to
// usbd_edpt_xfer() reach this through the linker (--wrap, CMakeLists.txt).
// Inert in UAC1 mode: EP 0x81 does not exist and usb_host_is_apple() is
// false. Called from the USB interrupt and from tud_task().
bool __real_usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer,
                           uint16_t total_bytes, bool is_isr);
bool __wrap_usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer,
                           uint16_t total_bytes, bool is_isr);

bool __wrap_usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer,
                           uint16_t total_bytes, bool is_isr)
{
    if (ep_addr == EPNUM_AUDIO_FB && total_bytes == 4 && usb_host_is_apple()) {
        uint32_t const fb = tu_le32toh(tu_unaligned_read32(buffer)) >> 2;
        buffer[0] = (uint8_t) fb;
        buffer[1] = (uint8_t) (fb >> 8);
        buffer[2] = (uint8_t) (fb >> 16);
        total_bytes = AUDIO_FB_EP_SIZE_APPLE;
    }
    return __real_usbd_edpt_xfer(rhport, ep_addr, buffer, total_bytes, is_isr);
}
