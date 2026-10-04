// Onboard LED: on while audio plays, off otherwise, two short flashes when
// the host enumerates the device.
#pragma once

#include <stdbool.h>
#include <stdint.h>

void status_led_init(void);

// events: USB_AUDIO_EVT_* bits taken this loop iteration.
void status_led_task(uint32_t events, bool stream_open, bool audio_flowing);

bool status_led_is_on(void);
