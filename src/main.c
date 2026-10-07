// usb2spdif - USB to S/PDIF bridge for the RP2350.

#include "pico/stdlib.h"
#include "pico/multicore.h"
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
#include "i2s/i2s_tx.h"
#include "spdif/spdif_rx.h"
#include "spdif/spdif_tx.h"
#include "usb/usb_audio.h"
#if USB2SPDIF_SELFTEST
#include "selftest/selftest.h"
#endif

// The internal pull-up (about 50 kOhm) charges the pad and a jumper wire
// (tens of pF) within a few us.
#define MODE_PIN_SETTLE_US 50u

static void jumper_init(unsigned pin) {
    gpio_init(pin);
    gpio_pull_up(pin);
}

// Grounded = 1.
static unsigned jumper_bit(unsigned pin, unsigned bit) {
    return gpio_get(pin) ? 0u : 1u << bit;
}

// Reads the configuration jumpers once: the USB audio class, the I2S
// output format and the dual-line filter's jumper code.
static void read_jumpers(usb_audio_mode_t *mode, i2s_format_t *format, unsigned *filter_code) {
    static const uint8_t pins[] = {
        UAC_MODE_PIN,    I2S_FORMAT_PIN0, I2S_FORMAT_PIN1, I2S_FORMAT_PIN2,
        I2S_FILTER_PIN0, I2S_FILTER_PIN1, I2S_FILTER_PIN2,
    };
    for (unsigned i = 0; i < sizeof(pins); i++) {
        jumper_init(pins[i]);
    }
    busy_wait_us_32(MODE_PIN_SETTLE_US);
    *mode = gpio_get(UAC_MODE_PIN) ? USB_AUDIO_MODE_UAC2 : USB_AUDIO_MODE_UAC1;
    *format = (i2s_format_t)(jumper_bit(I2S_FORMAT_PIN0, 0) | jumper_bit(I2S_FORMAT_PIN1, 1) |
                             jumper_bit(I2S_FORMAT_PIN2, 2));
    *filter_code = jumper_bit(I2S_FILTER_PIN0, 0) | jumper_bit(I2S_FILTER_PIN1, 1) |
                   jumper_bit(I2S_FILTER_PIN2, 2);
}

// Core 1: the I2S filters and, in the self-test build, the output
// decoders. Both are cooperative tasks.
static void core1_main(void) {
    for (;;) {
        i2s_tx_core1_task();
#if USB2SPDIF_SELFTEST
        selftest_task();
#endif
    }
}

int main(void) {
    usb_audio_mode_t mode;
    i2s_format_t i2s_format;
    unsigned filter_code;
    read_jumpers(&mode, &i2s_format, &filter_code);
    i2s_filter_type_t i2s_filter = i2s_filter_from_code(filter_code);
    // First: diag_init() derives its SysTick period from clk_sys.
    audio_clock_init(mode == USB_AUDIO_MODE_UAC1 ? AUDIO_CLOCK_FAST : AUDIO_CLOCK_ACCURATE);
    audio_clock_set_rate(48000);
    diag_init();
    monitor_init();
    monitor_set_config(i2s_format, filter_code);
    status_led_init();
    spdif_tx_init(SPDIF_PIN);
    i2s_tx_init(i2s_format, i2s_filter);
#if !USB2SPDIF_TEST_TONE
    spdif_rx_init(SPDIF_IN_COAX_PIN, SPDIF_IN_TOSLINK_PIN);
#endif
#if USB2SPDIF_SELFTEST
    selftest_init();
#endif
    multicore_launch_core1(core1_main);

#if USB2SPDIF_TEST_TONE
    spdif_tx_start(TEST_TONE_SAMPLE_RATE, 24, test_tone_source, 0);
#else
    audio_pipeline_init();
#endif

    usb_audio_init(mode);
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
        audio_pipeline_source_t source = {0};
#else
        diag_mark(DIAG_MARK_SPDIF_RX);
        spdif_rx_task();
        diag_mark(DIAG_MARK_PIPELINE);
        audio_pipeline_task();
        audio_pipeline_source_t source;
        audio_pipeline_get_source(&source);
#endif

        uint32_t ev = usb_audio_take_events();
        diag_mark(DIAG_MARK_LED);
        status_led_task(ev, &source);
        diag_mark(DIAG_MARK_LOG);
        monitor_task(ev);
        log_task();
    }
}
