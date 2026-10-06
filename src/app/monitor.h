// UART console log, the status line of the USB diagnostic string and the
// session string.
#pragma once

#include <stdint.h>

#include "i2s/i2s_tx.h"

// Hidden USB string index of the session string (next to STRID_DIAG), read
// by tools/monitor.py: "n<streams> fr<frames> sig<ms|-> up<ms>", all since
// boot: stream starts, frames moved into the output ring (wraps at 2^32),
// milliseconds since the last non-zero sample went out (- if none yet),
// uptime.
#define STRID_SESSION 0x23

// Hidden USB string index of the configuration string, read by
// tools/monitor.py; format in tools/diag.py.
#define STRID_CONFIG 0x24

// Initialises the console UART and prints the boot banner.
void monitor_init(void);

// events: USB_AUDIO_EVT_* bits taken this loop iteration.
void monitor_task(uint32_t events);

// The session string, formatted on each call. Main loop only.
const char *monitor_session_string(void);

// The jumper configuration for the configuration string: the I2S format
// and the filter jumpers' raw code (reserved codes included).
void monitor_set_config(i2s_format_t format, unsigned filter_code);

// The configuration string, formatted on each call. Main loop only.
const char *monitor_config_string(void);
