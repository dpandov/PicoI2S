#include "app/status_led.h"

#include "pico/stdlib.h"
#include "board_config.h"
#include "usb/usb_audio.h"

// The hold time bridges the pre-roll after an underrun to avoid flicker.
#define LED_AUDIO_HOLD_US     500000u
#define LED_FLASH_COUNT       2u
#define LED_FLASH_ON_US       80000u
#define LED_FLASH_PERIOD_US   200000u

static uint64_t last_audio_us;
static uint64_t flash_start_us;      // 0 = no flash sequence running

void status_led_init(void) {
    gpio_init(LOCK_LED_PIN);
    gpio_set_dir(LOCK_LED_PIN, GPIO_OUT);
}

static bool audio_active(uint64_t now, bool stream_open, bool audio_flowing) {
    if (stream_open && audio_flowing) {
        last_audio_us = now;
        return true;
    }
    return stream_open && last_audio_us != 0 && (now - last_audio_us) < LED_AUDIO_HOLD_US;
}

void status_led_task(uint32_t events, bool stream_open, bool audio_flowing) {
    uint64_t now = time_us_64();

    if (events & USB_AUDIO_EVT_MOUNTED) {
        flash_start_us = now;
    }

    if (flash_start_us != 0) {
        uint64_t t = now - flash_start_us;
        if (t < (uint64_t)LED_FLASH_COUNT * LED_FLASH_PERIOD_US) {
            gpio_put(LOCK_LED_PIN, (t % LED_FLASH_PERIOD_US) < LED_FLASH_ON_US);
            return;
        }
        flash_start_us = 0;
    }

    gpio_put(LOCK_LED_PIN, audio_active(now, stream_open, audio_flowing));
}

bool status_led_is_on(void) {
    return gpio_get_out_level(LOCK_LED_PIN);
}
