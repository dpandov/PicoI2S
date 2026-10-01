// UART console log, the status line of the USB diagnostic string and the
// session string.
#pragma once

#include <stdint.h>

// Hidden USB string index of the session string (next to STRID_DIAG), read
// by tools/monitor.py: "n<streams> fr<frames> sig<ms|-> up<ms>", all since
// boot: stream starts, frames moved into the output ring (wraps at 2^32),
// milliseconds since the last non-zero sample went out (- if none yet),
// uptime.
#define STRID_SESSION 0x23

// Initialises the console UART and prints the boot banner.
void monitor_init(void);

// events: USB_AUDIO_EVT_* bits taken this loop iteration.
void monitor_task(uint32_t events);

// The session string, formatted on each call. Main loop only.
const char *monitor_session_string(void);
