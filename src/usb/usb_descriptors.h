// USB descriptor constants shared by usb_descriptors.c, tusb_config.h and
// usb_audio.c.
#pragma once

#include <stdbool.h>

//--------------------------------------------------------------------+
// Device identity
//--------------------------------------------------------------------+

// pid.codes test VID/PIDs, for private testing only. Dedicated PIDs are
// required for public release. UAC1 mode has its own PID because Windows
// caches the driver binding per VID/PID.
#define USB_VID                 0x1209
#define USB_PID                 0x0001
#define USB_PID_UAC1            0x0002

// Bump on every descriptor change: Windows caches descriptors per
// VID/PID/bcdDevice.
#define USB_BCD_DEVICE          0x0103

#define USB_MANUFACTURER_STR    "usb2spdif"
#define USB_PRODUCT_STR         "USB to S/PDIF"
// UAC1 mode: replaces the product and the audio function string (hosts
// name the device after either). The host tools match both names.
#define USB_PRODUCT_STR_UAC1    "USB to SPDIF"

//--------------------------------------------------------------------+
// Interfaces
//--------------------------------------------------------------------+

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_RESET,            // picotool reset interface (SDK pico_usb_reset)
    ITF_NUM_TOTAL
};

// Streaming interface alternate settings.
#define AS_ALT_ZERO_BW          0
#define AS_ALT_16BIT            1   // 16-bit samples in 2-byte subslots
#define AS_ALT_24BIT            2   // 24-bit samples, MSB-aligned in 4-byte subslots

#define AUDIO_N_CHANNELS        2
#define AUDIO_16BIT_SUBSLOT     2
#define AUDIO_16BIT_RESOLUTION  16
#define AUDIO_24BIT_SUBSLOT     4
#define AUDIO_24BIT_RESOLUTION  24

#define AUDIO_MAX_SAMPLE_RATE   96000
#define UAC1_MAX_SAMPLE_RATE    48000

//--------------------------------------------------------------------+
// UAC2 entities: Clock -> Input Terminal -> Feature Unit -> Output Terminal
//--------------------------------------------------------------------+

#define UAC2_ENTITY_INPUT_TERMINAL   0x01   // USB streaming
#define UAC2_ENTITY_FEATURE_UNIT     0x02   // master mute and volume
#define UAC2_ENTITY_OUTPUT_TERMINAL  0x03   // S/PDIF interface
#define UAC2_ENTITY_CLOCK            0x04   // internal programmable clock

// "S/PDIF interface" external terminal type (same code in UAC1 and UAC2);
// TinyUSB has no enum for it.
#define TERM_TYPE_EXT_SPDIF          0x0605

//--------------------------------------------------------------------+
// UAC1 entities: Input Terminal -> Feature Unit -> Output Terminal
//--------------------------------------------------------------------+

#define UAC1_ENTITY_INPUT_TERMINAL   0x01   // USB streaming
#define UAC1_ENTITY_FEATURE_UNIT     0x02   // master mute and volume
#define UAC1_ENTITY_OUTPUT_TERMINAL  0x03   // S/PDIF interface (0x0605)

//--------------------------------------------------------------------+
// Endpoints
//--------------------------------------------------------------------+

#define EPNUM_AUDIO_OUT         0x01    // isochronous data
#define EPNUM_AUDIO_FB          0x81    // isochronous feedback

// Feedback endpoint size on Full Speed (UAC2 only). Windows only accepts
// 4 bytes with a 16.16 value (a 3-byte endpoint fails with Code 10). Apple
// hosts require 3 bytes with a 10.14 value. Apple hosts get their own
// configuration.
#define AUDIO_FB_EP_SIZE        4
#define AUDIO_FB_EP_SIZE_APPLE  3

// True when enumeration identified an Apple host. Always false in UAC1
// mode, which has no feedback endpoint.
bool usb_host_is_apple(void);

//--------------------------------------------------------------------+
// String indices
//--------------------------------------------------------------------+

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_AUDIO_FUNCTION,
    STRID_RESET,
    STRID_COUNT
};

//--------------------------------------------------------------------+
// Descriptor lengths
//--------------------------------------------------------------------+

// Expanded only where tusb.h is already included.

// CS AC entities after the header: clock source, input terminal, feature
// unit (master and two channels), output terminal.
#define USB2SPDIF_AC_ENTITIES_LEN (TUD_AUDIO20_DESC_CLK_SRC_LEN \
    + TUD_AUDIO20_DESC_INPUT_TERM_LEN                            \
    + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(AUDIO_N_CHANNELS)        \
    + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN)

// One streaming alt setting (alt 1 or 2).
#define USB2SPDIF_AS_ALT_STREAM_DESC_LEN (TUD_AUDIO20_DESC_STD_AS_LEN \
    + TUD_AUDIO20_DESC_CS_AS_INT_LEN                                  \
    + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN                              \
    + TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN                              \
    + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN                               \
    + TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP_LEN)

// The whole UAC2 function, IAD included.
#define USB2SPDIF_AUDIO_DESC_LEN (TUD_AUDIO20_DESC_IAD_LEN \
    + TUD_AUDIO20_DESC_STD_AC_LEN                          \
    + TUD_AUDIO20_DESC_CS_AC_LEN                           \
    + USB2SPDIF_AC_ENTITIES_LEN                            \
    + TUD_AUDIO20_DESC_STD_AS_LEN                          \
    + 2 * USB2SPDIF_AS_ALT_STREAM_DESC_LEN)

// UAC1 Feature Unit with 1-byte controls: master and two channels.
#define USB2SPDIF_UAC1_FEATURE_UNIT_LEN (7 + (AUDIO_N_CHANNELS + 1))

// UAC1 CS AC entities after the header: input terminal, feature unit,
// output terminal.
#define USB2SPDIF_UAC1_AC_ENTITIES_LEN (TUD_AUDIO10_DESC_INPUT_TERM_LEN \
    + USB2SPDIF_UAC1_FEATURE_UNIT_LEN                                  \
    + TUD_AUDIO10_DESC_OUTPUT_TERM_LEN)

// 32, 44.1 and 48 kHz.
#define USB2SPDIF_UAC1_N_RATES  3

// The whole UAC1 function, no IAD: AC header naming one streaming
// interface, which has alt 0 and one 16-bit alt setting.
#define USB2SPDIF_UAC1_AUDIO_DESC_LEN (TUD_AUDIO10_DESC_STD_AC_LEN \
    + TUD_AUDIO10_DESC_CS_AC_LEN(1)                                 \
    + USB2SPDIF_UAC1_AC_ENTITIES_LEN                                \
    + 2 * TUD_AUDIO10_DESC_STD_AS_LEN                               \
    + TUD_AUDIO10_DESC_CS_AS_INT_LEN                                \
    + TUD_AUDIO10_DESC_TYPE_I_FORMAT_LEN(USB2SPDIF_UAC1_N_RATES)    \
    + TUD_AUDIO10_DESC_STD_AS_ISO_EP_LEN                            \
    + TUD_AUDIO10_DESC_CS_AS_ISO_EP_LEN)
