// usb2spdif - USB to S/PDIF bridge for the Raspberry Pi Pico 2.

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include "board_config.h"
#include "app/audio_pipeline.h"
#include "app/log.h"
#include "app/monitor.h"
#include "app/status_led.h"
#include "app/test_tone.h"
#include "clock/audio_clock.h"
#include "diag/diag.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"
#if USB2SPDIF_SELFTEST
#include "selftest/selftest.h"
#endif

// The internal pull-up (about 50 kOhm) charges the pad and a jumper wire
// (tens of pF) within a few us.
#define MODE_PIN_SETTLE_US 50u

static usb_audio_mode_t read_mode_pin(void) {
    gpio_init(UAC_MODE_PIN);
    gpio_pull_up(UAC_MODE_PIN);
    busy_wait_us_32(MODE_PIN_SETTLE_US);
    return gpio_get(UAC_MODE_PIN) ? USB_AUDIO_MODE_UAC2 : USB_AUDIO_MODE_UAC1;
}

int main(void) {
    // First: diag_init() derives its SysTick period from clk_sys.
    audio_clock_set_family(AUDIO_FAMILY_48K);
    diag_init();
    monitor_init();
    status_led_init();
    spdif_tx_init(SPDIF_PIN);
#if USB2SPDIF_SELFTEST
    selftest_init();
#endif

#if USB2SPDIF_TEST_TONE
    spdif_tx_start(TEST_TONE_SAMPLE_RATE, 24, test_tone_source);
#else
    audio_pipeline_init();
#endif

    usb_audio_init(read_mode_pin());
    const tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    tusb_init(0, &dev_init);

    while (true) {
        diag_loop();
        diag_mark(DIAG_MARK_TUD_TASK);
        tud_task();
#if USB2SPDIF_TEST_TONE
        bool playing = false;
#else
        diag_mark(DIAG_MARK_PIPELINE);
        audio_pipeline_task();
        bool playing = audio_pipeline_playing();
#endif

        uint32_t ev = usb_audio_take_events();
        bool stream_open = usb_audio_link_state() == USB_AUDIO_STREAMING;
        diag_mark(DIAG_MARK_LED);
        status_led_task(ev, stream_open, playing);
        diag_mark(DIAG_MARK_LOG);
        monitor_task(ev);
        log_task();
    }
}
