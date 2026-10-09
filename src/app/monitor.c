#include "app/monitor.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "app/audio_pipeline.h"
#include "app/log.h"
#include "app/status_led.h"
#include "diag/diag.h"
#include "i2s/i2s_tx.h"
#include "spdif/spdif_rx.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"
#include "usb/usb_descriptors.h"

void monitor_init(void) {
    log_init();
    log_printf("\nusb2spdif\n");
    log_printf("clk_sys %lu, clk_peri %lu, clk_usb %lu\n",
               (unsigned long)clock_get_hz(clk_sys),
               (unsigned long)clock_get_hz(clk_peri),
               (unsigned long)clock_get_hz(clk_usb));
}

static void log_format(const char *what) {
    usb_audio_format_t f = usb_audio_format();
    log_printf("%s %lu Hz %u-bit\n", what, (unsigned long)f.sample_rate, f.resolution);
}

static void log_events(uint32_t ev) {
    if (ev & USB_AUDIO_EVT_MOUNTED)
        log_printf("usb: mounted, UAC%d\n", usb_audio_mode() == USB_AUDIO_MODE_UAC1 ? 1 : 2);
    if (ev & USB_AUDIO_EVT_UNMOUNTED)    log_printf("usb: unmounted\n");
    if (ev & USB_AUDIO_EVT_RATE_CHANGED) log_format("usb: rate");
    if (ev & USB_AUDIO_EVT_STREAM_START) log_format("usb: stream start");
    if (ev & USB_AUDIO_EVT_STREAM_STOP)  log_printf("usb: stream stop\n");
}

static const char *input_name(spdif_rx_input_t input) {
    return input == SPDIF_RX_COAX ? "coax" : (input == SPDIF_RX_TOSLINK ? "toslink" : "none");
}

static void receiver_stats(spdif_rx_stats_t *st) {
#if USB2SPDIF_TEST_TONE
    *st = (spdif_rx_stats_t){0};
#else
    spdif_rx_get_stats(st);
#endif
}

// Receiver changes: lock per input, the selected input and its rate,
// channel-status flags.
static void log_input(void) {
    static spdif_rx_status_t prev;
    audio_pipeline_source_t src;
    audio_pipeline_get_source(&src);
    const spdif_rx_status_t *st = &src.input;
    if (st->locked_coax != prev.locked_coax)
        log_printf("input: coax %s\n", st->locked_coax ? "locked" : "unlocked");
    if (st->locked_toslink != prev.locked_toslink)
        log_printf("input: toslink %s\n", st->locked_toslink ? "locked" : "unlocked");
    if (st->input != prev.input || st->sample_rate != prev.sample_rate) {
        if (st->input == SPDIF_RX_NONE)
            log_printf("input: none selected\n");
        else
            log_printf("input: %s selected, %lu Hz %ld ppm\n", input_name(st->input),
                       (unsigned long)st->sample_rate, (long)st->rate_ppm);
    }
    bool emphasis = st->cs_valid && st->emphasis;
    if (emphasis != (prev.cs_valid && prev.emphasis))
        log_printf("input: pre-emphasis %s\n", emphasis ? "on" : "off");
    bool data = st->cs_valid && !st->pcm;
    if (data != (prev.cs_valid && !prev.pcm))
        log_printf("input: %s\n", data ? "non-PCM, muted" : "PCM");
    prev = *st;
}

// Keeps the status line within its 95 characters.
static unsigned long clamp_cnt(uint32_t v, uint32_t max) {
    return v > max ? (unsigned long)max : (unsigned long)v;
}

// ppb -> ppm, rounded.
static long trim_ppm(int32_t ppb) {
    return (long)((ppb + (ppb < 0 ? -500 : 500)) / 1000);
}

// Format documented in tools/diag.py. At most 93 characters untrimmed
// (rmin and fmax have at most 3 and 4 digits). UAC1 mode shows '1' in place
// of the Apple host flag. While the output follows an external clock (UAC1
// mode, or the S/PDIF input playing) the rate trim and servo lock come
// before the volume; counters stop two digits earlier so that it stays
// within 94.
static void update_diag_status(void) {
    static uint64_t last_us;
    uint64_t now = time_us_64();
    if (now - last_us < 250000u) {
        return;
    }
    last_us = now;

    usb_audio_format_t f = usb_audio_format();
    usb_audio_stats_t s;
    usb_audio_get_stats(&s);
    audio_pipeline_stats_t ps;
    audio_pipeline_get_stats(&ps);
    static const char link_chr[] = {'D', 'I', 'S'};
    bool uac1 = usb_audio_mode() == USB_AUDIO_MODE_UAC1;
    audio_pipeline_source_t src;
    audio_pipeline_get_source(&src);
    bool trimmed = uac1 || src.heard == AUDIO_SOURCE_COAX || src.heard == AUDIO_SOURCE_TOSLINK;
    uint32_t max = trimmed ? 9999u : 999999u;
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
                     "%c%c %lu/%u ff%u ur%lu run%d led%d tx%lu ov%lu lt%lu rmin%lu fmax%lu lp%lu",
                     link_chr[usb_audio_link_state()],
                     uac1 ? '1' : (usb_host_is_apple() ? 'a' : 'o'),
                     (unsigned long)f.sample_rate, f.resolution, s.fifo_level,
                     clamp_cnt(ps.underruns, max), audio_pipeline_playing() ? 1 : 0,
                     status_led_is_on() ? 1 : 0, (unsigned long)spdif_tx_sample_rate(),
                     clamp_cnt(ps.overruns, max), clamp_cnt(ps.late_irqs, max),
                     clamp_cnt(ps.ring_min, max), clamp_cnt(ps.fifo_max, max),
                     clamp_cnt(ps.loop_max_us, max));
    if (trimmed && n > 0 && (size_t)n < sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, " tr%ld%c", trim_ppm(ps.trim_ppb),
                      ps.clock_locked ? 'L' : 'U');
    }
    usb_audio_volume_t v = usb_audio_volume();
    if (n > 0 && (size_t)n < sizeof(buf)) {
        if (v.mute) {
            snprintf(buf + n, sizeof(buf) - (size_t)n, " vM");
        } else {
            snprintf(buf + n, sizeof(buf) - (size_t)n, " v%u", v.att_db);
        }
    }
    diag_set_app_status(buf);
}

static void log_stats(void) {
    static uint64_t last_us;
    static uint32_t last_rx;
    uint64_t now = time_us_64();
    if (now - last_us < 5000000u) {
        return;
    }
    usb_audio_stats_t st;
    usb_audio_get_stats(&st);
    if (usb_audio_link_state() == USB_AUDIO_STREAMING && last_us != 0) {
        uint32_t bps = (uint32_t)(((uint64_t)(st.rx_bytes - last_rx) * 1000000u) / (now - last_us));
        audio_pipeline_stats_t ps;
        audio_pipeline_get_stats(&ps);
        log_printf("stat: %lu B/s fifo %u/%u max %lu ur %lu/%lu ov %lu/%lu bad %lu "
                   "lt %lu rmin %lu lp %lu drop %lu trim %ld%s\n",
                   (unsigned long)bps, st.fifo_level, st.fifo_depth,
                   (unsigned long)ps.fifo_max, (unsigned long)ps.underruns,
                   (unsigned long)ps.underruns_total, (unsigned long)ps.overruns,
                   (unsigned long)ps.overruns_total, (unsigned long)st.bad_packets,
                   (unsigned long)ps.late_irqs,
                   (unsigned long)ps.ring_min, (unsigned long)ps.loop_max_us,
                   (unsigned long)log_dropped(), (long)ps.trim_ppb,
                   ps.clock_locked ? " locked" : "");
    }
    audio_pipeline_source_t src;
    audio_pipeline_get_source(&src);
    if (src.heard == AUDIO_SOURCE_COAX || src.heard == AUDIO_SOURCE_TOSLINK) {
        audio_pipeline_stats_t ps;
        audio_pipeline_get_stats(&ps);
        spdif_rx_stats_t rs;
        receiver_stats(&rs);
        log_printf("stat: in %s %lu Hz %ld ppm lvl %lu/%lu ur %lu lt %lu rmin %lu lp %lu "
                   "par %lu frm %lu ovf %lu trim %ld%s\n",
                   input_name(src.input.input), (unsigned long)src.input.sample_rate,
                   (long)src.input.rate_ppm, (unsigned long)spdif_rx_level(),
                   (unsigned long)spdif_rx_depth(), (unsigned long)ps.underruns,
                   (unsigned long)ps.late_irqs, (unsigned long)ps.ring_min,
                   (unsigned long)ps.loop_max_us, (unsigned long)rs.err_parity,
                   (unsigned long)rs.err_frame, (unsigned long)rs.overflows,
                   (long)ps.trim_ppb, ps.clock_locked ? " locked" : "");
    }
    last_us = now;
    last_rx = st.rx_bytes;
}

void monitor_task(uint32_t events) {
    log_events(events);
    log_input();
    update_diag_status();
    log_stats();
}

// Format documented in tools/diag.py: "n<streams> fr<frames> sig<ms|-> up<ms>
// os<late> src<u|c|t|-> in<c|t|->/<c|-><t|-> ir<rate><+|-><ppm>|ir-
// cs<bits>[e][n]|cs- rx<locks>/<unlocks>/<parity>/<frame>/<overflows>", at
// most 123 characters (counters from os on capped at 99999, ppm at 9999).
const char *monitor_session_string(void) {
    static char buf[128];
    static const char source_chr[] = {'-', 'u', 'c', 't'};
    static const char input_chr[] = {'-', 'c', 't'};
    audio_pipeline_session_t s;
    audio_pipeline_get_session(&s);
    audio_pipeline_source_t src;
    audio_pipeline_get_source(&src);
    const spdif_rx_status_t *in = &src.input;
    spdif_rx_stats_t rs;
    receiver_stats(&rs);
    const uint32_t max = 99999u;

    int n = snprintf(buf, sizeof(buf), "n%lu fr%lu sig", (unsigned long)s.streams,
                     (unsigned long)s.frames);
    if (n > 0 && (size_t)n < sizeof(buf)) {
        if (s.signal_age_ms == UINT32_MAX) {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, "-");
        } else {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%lu", (unsigned long)s.signal_age_ms);
        }
    }
    if (n > 0 && (size_t)n < sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, " up%lu os%lu src%c in%c/%c%c",
                      (unsigned long)(time_us_64() / 1000u), clamp_cnt(i2s_tx_late(), max),
                      source_chr[src.heard], input_chr[in->input],
                      in->locked_coax ? 'c' : '-', in->locked_toslink ? 't' : '-');
    }
    if (n > 0 && (size_t)n < sizeof(buf)) {
        if (in->input != SPDIF_RX_NONE && in->sample_rate != 0) {
            int32_t ppm = in->rate_ppm;
            ppm = ppm > 9999 ? 9999 : (ppm < -9999 ? -9999 : ppm);
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " ir%lu%c%ld",
                          (unsigned long)in->sample_rate, ppm < 0 ? '-' : '+',
                          (long)(ppm < 0 ? -ppm : ppm));
        } else {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " ir-");
        }
    }
    if (n > 0 && (size_t)n < sizeof(buf)) {
        if (in->input != SPDIF_RX_NONE && in->cs_valid) {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " cs%u%s%s", in->word_bits,
                          in->emphasis ? "e" : "", in->pcm ? "" : "n");
        } else {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " cs-");
        }
    }
    if (n > 0 && (size_t)n < sizeof(buf)) {
        snprintf(buf + n, sizeof(buf) - (size_t)n, " rx%lu/%lu/%lu/%lu/%lu",
                 clamp_cnt(rs.locks, max), clamp_cnt(rs.unlocks, max),
                 clamp_cnt(rs.err_parity, max), clamp_cnt(rs.err_frame, max),
                 clamp_cnt(rs.overflows, max));
    }
    return buf;
}

static i2s_format_t config_format;
static unsigned config_filter_code;

void monitor_set_config(i2s_format_t format, unsigned filter_code) {
    config_format = format;
    config_filter_code = filter_code;
}

// Three jumper bits, the highest first (as the README tables).
static void bits3(char *s, unsigned code) {
    for (unsigned i = 0; i < 3u; i++)
        s[i] = (code >> (2u - i)) & 1u ? '1' : '0';
    s[3] = '\0';
}

// Format documented in tools/diag.py, at most 105 characters. The filter
// is the jumper setting even where it is not used (I2S / LJ / RJ).
const char *monitor_config_string(void) {
    static char buf[112];
#if USB2SPDIF_BOARD_RP2350_ONE
    const char *board = "rp2350-one";
#else
    const char *board = "pico2";
#endif
#if USB2SPDIF_TEST_TONE && USB2SPDIF_SELFTEST
    const char *build = "tone+selftest";
#elif USB2SPDIF_TEST_TONE
    const char *build = "tone";
#elif USB2SPDIF_SELFTEST
    const char *build = "selftest";
#else
    const char *build = "normal";
#endif
    bool uac1 = usb_audio_mode() == USB_AUDIO_MODE_UAC1;
    char fmt_bits[4], flt_bits[4];
    bits3(fmt_bits, (unsigned)config_format);
    bits3(flt_bits, config_filter_code);
    snprintf(buf, sizeof(buf), "board=%s fw=%04x build=%s mode=%s format=%s filter=%s jumpers=%c/%s/%s",
             board, USB_BCD_DEVICE, build, uac1 ? "uac1" : "uac2", i2s_format_name(config_format),
             i2s_filter_name(i2s_filter_from_code(config_filter_code)), uac1 ? '1' : '0', fmt_bits,
             flt_bits);
    return buf;
}
