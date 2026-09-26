// UART console log and the status line of the USB diagnostic string.
#pragma once

#include <stdint.h>

// Initialises the console UART and prints the boot banner.
void monitor_init(void);

// events: USB_AUDIO_EVT_* bits taken this loop iteration.
void monitor_task(uint32_t events);
