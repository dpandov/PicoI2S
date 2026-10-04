// TinyUSB configuration.
#pragma once

#include "usb/usb_descriptors.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined (the Pico SDK does this)
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS             OPT_OS_PICO
#endif

// Debug output disturbs isochronous timing.
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG          0
#endif

#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE  64

#define CFG_TUD_AUDIO           1
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0   // the reset interface is an app driver

//--------------------------------------------------------------------+
// Audio class
//--------------------------------------------------------------------+

// Largest reply is the clock RANGE (see usb_audio.c).
#define CFG_TUD_AUDIO_CTRL_BUF_SZ           64

// Sized for the largest alt setting of either mode: UAC2 96 kHz, 4-byte
// subslots, stereo.
#define CFG_TUD_AUDIO_ENABLE_EP_OUT         1
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX  TUD_AUDIO_EP_SIZE(false, AUDIO_MAX_SAMPLE_RATE, AUDIO_24BIT_SUBSLOT, AUDIO_N_CHANNELS)

// 12 max-size packets. Feedback (UAC2) or the clock servo (UAC1) holds it
// half full (6 ms at 96 kHz / 24-bit, 24 ms at 48 kHz / 16-bit); the upper
// half takes the pre-roll (see audio_pipeline.c). Must be a multiple of
// both frame sizes (4, 8).
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (12 * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX)

#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP    1

#ifdef __cplusplus
}
#endif
