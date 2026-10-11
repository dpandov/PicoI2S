// Onboard LED: shows the source being heard, off otherwise, two short
// flashes when the host enumerates the device. RP2350-One (WS2812): red for
// USB, green for the S/PDIF input, yellow for the input with pre-emphasis
// (colours in board_config.h). Pico 2: on for any source, and an external
// LED on INPUT_LOCK_LED_PIN while either S/PDIF input is locked.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app/audio_pipeline.h"

void status_led_init(void);

// events: USB_AUDIO_EVT_* bits taken this loop iteration; source: from
// audio_pipeline_get_source().
void status_led_task(uint32_t events, const audio_pipeline_source_t *source);

bool status_led_is_on(void);
