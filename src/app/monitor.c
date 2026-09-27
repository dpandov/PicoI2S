#include "app/monitor.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "board_config.h"
#include "app/audio_pipeline.h"
#include "app/status_led.h"
#include "diag/diag.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"
#include "usb/usb_descriptors.h"

// stdio blocks while the UART FIFO is full: keep lines short and rare.

void monitor_init(void) {
    stdio_uart_init_full(CONSOLE_UART, CONSOLE_BAUD, CONSOLE_UART_TX, CONSOLE_UART_RX);
    printf("\nusb2spdif\n");
    printf("clk_sys %lu, clk_peri %lu, clk_usb %lu\n",
           (unsigned long)clock_get_hz(clk_sys),
           (unsigned long)clock_get_hz(clk_peri),
           (unsigned long)clock_get_hz(clk_usb));
}

static void log_format(const char *what) {
    usb_audio_format_t f = usb_audio_format();
    printf("%s %lu Hz %u-bit\n", what, (unsigned long)f.sample_rate, f.resolution);
}

static void log_events(uint32_t ev) {
    if (ev & USB_AUDIO_EVT_MOUNTED)      printf("usb: mounted\n");
    if (ev & USB_AUDIO_EVT_UNMOUNTED)    printf("usb: unmounted\n");
    if (ev & USB_AUDIO_EVT_RATE_CHANGED) log_format("usb: rate");
    if (ev & USB_AUDIO_EVT_STREAM_START) log_format("usb: stream start");
    if (ev & USB_AUDIO_EVT_STREAM_STOP)  printf("usb: stream stop\n");
}

// Format documented in tools/diag.py.
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
    static const char link_chr[] = {'D', 'I', 'S'};
    char buf[49];
    snprintf(buf, sizeof(buf), "%c%c %lu/%u ff%u ur%lu run%d led%d tx%lu",
             link_chr[usb_audio_link_state()], usb_host_is_apple() ? 'a' : 'o',
             (unsigned long)f.sample_rate, f.resolution, s.fifo_level,
             (unsigned long)audio_pipeline_underruns(), audio_pipeline_playing() ? 1 : 0,
             status_led_is_on() ? 1 : 0, (unsigned long)spdif_tx_sample_rate());
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
        printf("stat: %lu B/s fifo %u/%u ur %lu\n", (unsigned long)bps,
               st.fifo_level, st.fifo_depth, (unsigned long)audio_pipeline_underruns());
    }
    last_us = now;
    last_rx = st.rx_bytes;
}

void monitor_task(uint32_t events) {
    log_events(events);
    update_diag_status();
    log_stats();
}
