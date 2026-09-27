#include "app/monitor.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "app/audio_pipeline.h"
#include "app/log.h"
#include "app/status_led.h"
#include "diag/diag.h"
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

// Keeps the status line within its 95 characters.
static unsigned long clamp_cnt(uint32_t v, uint32_t max) {
    return v > max ? (unsigned long)max : (unsigned long)v;
}

// ppb -> ppm, rounded.
static long trim_ppm(int32_t ppb) {
    return (long)((ppb + (ppb < 0 ? -500 : 500)) / 1000);
}

// Format documented in tools/diag.py. At most 89 characters in UAC2 mode.
// UAC1 mode shows '1' in place of the Apple host flag and appends the rate
// trim and servo lock; counters stop one digit earlier so that it stays
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
    uint32_t max = uac1 ? 99999u : 999999u;
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
    if (uac1 && n > 0 && (size_t)n < sizeof(buf)) {
        snprintf(buf + n, sizeof(buf) - (size_t)n, " tr%ld%c", trim_ppm(ps.trim_ppb),
                 ps.clock_locked ? 'L' : 'U');
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
    last_us = now;
    last_rx = st.rx_bytes;
}

void monitor_task(uint32_t events) {
    log_events(events);
    update_diag_status();
    log_stats();
}
