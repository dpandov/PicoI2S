// USB descriptors.
//
// Interfaces: 0 Audio Control and 1 Audio Streaming (UAC2, one IAD),
// 2 picotool reset (vendor). Topology: Clock -> USB streaming Input
// Terminal -> S/PDIF Output Terminal. The BOS / MS OS 2.0 descriptors come
// from the SDK's pico_usb_reset.

#include <stddef.h>
#include <stdint.h>

#include "tusb.h"
#include "pico/unique_id.h"
#include "pico/usb_reset.h"

#include "usb/usb_descriptors.h"
#include "diag/diag.h"

TU_VERIFY_STATIC(ITF_NUM_AUDIO_STREAMING == ITF_NUM_AUDIO_CONTROL + 1,
                 "UAC2 interfaces must be contiguous for the IAD");

// Windows binds WinUSB to the interface named by the MS OS 2.0 descriptor.
#ifndef PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF
#error PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF must be defined (CMakeLists.txt)
#endif
TU_VERIFY_STATIC(ITF_NUM_RESET == PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF,
                 "MS OS 2.0 descriptor must target the reset interface");

TU_VERIFY_STATIC((EPNUM_AUDIO_OUT & TUSB_DIR_IN_MASK) == 0, "audio data EP must be OUT");
TU_VERIFY_STATIC((EPNUM_AUDIO_FB & TUSB_DIR_IN_MASK) != 0, "feedback EP must be IN");

//--------------------------------------------------------------------+
// Device descriptor
//--------------------------------------------------------------------+

// USB 2.1 so hosts request the BOS descriptor; Windows needs it to bind
// WinUSB to the reset interface. The device is still Full Speed.
#define USB_BCD_USB_WITH_BOS    0x0210

static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD_USB_WITH_BOS,

    // Required by the IAD ECN for composite devices.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = USB_BCD_DEVICE,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 1,
};

static bool host_decided;

uint8_t const *tud_descriptor_device_cb(void)
{
    host_decided = false;   // enumeration (re)starts
    return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// UAC2 function
//--------------------------------------------------------------------+

#define AUDIO_STEREO_CHANNEL_CONFIG \
    (AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT)

#define AUDIO_FUNCTION_CATEGORY     AUDIO_FUNC_CONVERTER

#define AUDIO_CLOCK_CONTROLS \
    ((AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS) | \
     (AUDIO_CTRL_R  << AUDIO_CLOCK_SOURCE_CTRL_CLK_VAL_POS))

#define AUDIO_NO_ASSOC_TERMINAL     0x00

// Asynchronous: the device is the clock master and reports its rate
// through the feedback endpoint.
#define AUDIO_DATA_EP_ATTR \
    ((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS | \
     (uint8_t)TUSB_ISO_EP_ATT_DATA)

#define AUDIO_EP_INTERVAL_FS        1   // every 1 ms frame

// 96 kHz plus one extra frame for rate adaptation.
#define AUDIO_EP_SIZE(_subslot) \
    TUD_AUDIO_EP_SIZE(AUDIO_MAX_SAMPLE_RATE, _subslot, AUDIO_N_CHANNELS)

TU_VERIFY_STATIC(AUDIO_EP_SIZE(AUDIO_16BIT_SUBSLOT) == 388, "unexpected 16-bit EP size");
TU_VERIFY_STATIC(AUDIO_EP_SIZE(AUDIO_24BIT_SUBSLOT) == 776, "unexpected 24-bit EP size");
TU_VERIFY_STATIC(AUDIO_EP_SIZE(AUDIO_24BIT_SUBSLOT) == CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX,
                 "largest alt setting must match CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX");
TU_VERIFY_STATIC(AUDIO_EP_SIZE(AUDIO_24BIT_SUBSLOT) <= 1023, "FS iso packet limit exceeded");

// No Feature Unit on purpose: without device volume, the host applies
// volume in software and exclusive-mode playback stays bit-perfect.
#define AUDIO_AC_ENTITIES                                                     \
    TUD_AUDIO_DESC_CLK_SRC(                                                   \
        /*_clkid*/     UAC2_ENTITY_CLOCK,                                     \
        /*_attr*/      AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK,                    \
        /*_ctrl*/      AUDIO_CLOCK_CONTROLS,                                  \
        /*_assocTerm*/ AUDIO_NO_ASSOC_TERMINAL,                               \
        /*_stridx*/    0x00),                                                 \
    TUD_AUDIO_DESC_INPUT_TERM(                                                \
        /*_termid*/           UAC2_ENTITY_INPUT_TERMINAL,                     \
        /*_termtype*/         AUDIO_TERM_TYPE_USB_STREAMING,                  \
        /*_assocTerm*/        AUDIO_NO_ASSOC_TERMINAL,                        \
        /*_clkid*/            UAC2_ENTITY_CLOCK,                              \
        /*_nchannelslogical*/ AUDIO_N_CHANNELS,                               \
        /*_channelcfg*/       AUDIO_STEREO_CHANNEL_CONFIG,                    \
        /*_idxchannelnames*/  0x00,                                           \
        /*_ctrl*/             AUDIO_CTRL_NONE,                                \
        /*_stridx*/           0x00),                                          \
    TUD_AUDIO_DESC_OUTPUT_TERM(                                               \
        /*_termid*/    UAC2_ENTITY_OUTPUT_TERMINAL,                           \
        /*_termtype*/  UAC2_TERM_TYPE_EXT_SPDIF,                              \
        /*_assocTerm*/ AUDIO_NO_ASSOC_TERMINAL,                               \
        /*_srcid*/     UAC2_ENTITY_INPUT_TERMINAL,                            \
        /*_clkid*/     UAC2_ENTITY_CLOCK,                                     \
        /*_ctrl*/      AUDIO_CTRL_NONE,                                       \
        /*_stridx*/    0x00)

// A streaming alt setting with data and feedback endpoints.
#define AUDIO_AS_STREAM_ALT(_alt, _subslot, _resolution, _fbsize)           \
    TUD_AUDIO_DESC_STD_AS_INT(                                                \
        /*_itfnum*/ ITF_NUM_AUDIO_STREAMING,                                  \
        /*_altset*/ _alt,                                                     \
        /*_nEPs*/   2,                                                        \
        /*_stridx*/ 0x00),                                                    \
    TUD_AUDIO_DESC_CS_AS_INT(                                                 \
        /*_termid*/            UAC2_ENTITY_INPUT_TERMINAL,                    \
        /*_ctrl*/              AUDIO_CTRL_NONE,                               \
        /*_formattype*/        AUDIO_FORMAT_TYPE_I,                           \
        /*_formats*/           AUDIO_DATA_FORMAT_TYPE_I_PCM,                  \
        /*_nchannelsphysical*/ AUDIO_N_CHANNELS,                              \
        /*_channelcfg*/        AUDIO_STEREO_CHANNEL_CONFIG,                   \
        /*_stridx*/            0x00),                                         \
    TUD_AUDIO_DESC_TYPE_I_FORMAT(_subslot, _resolution),                      \
    TUD_AUDIO_DESC_STD_AS_ISO_EP(                                             \
        /*_ep*/        EPNUM_AUDIO_OUT,                                       \
        /*_attr*/      AUDIO_DATA_EP_ATTR,                                    \
        /*_maxEPsize*/ AUDIO_EP_SIZE(_subslot),                               \
        /*_interval*/  AUDIO_EP_INTERVAL_FS),                                 \
    /* Short packets are normal (44/45 frames per ms at 44.1 kHz). */        \
    TUD_AUDIO_DESC_CS_AS_ISO_EP(                                              \
        /*_attr*/          AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,    \
        /*_ctrl*/          AUDIO_CTRL_NONE,                                   \
        /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, \
        /*_lockdelay*/     0x0000),                                           \
    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(                                          \
        /*_ep*/       EPNUM_AUDIO_FB,                                         \
        /*_epsize*/   _fbsize,                                                \
        /*_interval*/ AUDIO_EP_INTERVAL_FS)

// _fbsize: AUDIO_FB_EP_SIZE or AUDIO_FB_EP_SIZE_APPLE.
#define USB2SPDIF_AUDIO_FUNCTION_DESCRIPTOR(_fbsize)                          \
    TUD_AUDIO_DESC_IAD(                                                       \
        /*_firstitf*/ ITF_NUM_AUDIO_CONTROL,                                  \
        /*_nitfs*/    2,                                                      \
        /*_stridx*/   STRID_AUDIO_FUNCTION),                                  \
    TUD_AUDIO_DESC_STD_AC(                                                    \
        /*_itfnum*/ ITF_NUM_AUDIO_CONTROL,                                    \
        /*_nEPs*/   0,                                                        \
        /*_stridx*/ STRID_AUDIO_FUNCTION),                                    \
    TUD_AUDIO_DESC_CS_AC(                                                     \
        /*_bcdADC*/   0x0200,                                                 \
        /*_category*/ AUDIO_FUNCTION_CATEGORY,                                \
        /*_totallen*/ USB2SPDIF_AC_ENTITIES_LEN,                              \
        /*_ctrl*/     (AUDIO_CTRL_NONE << AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS)), \
    AUDIO_AC_ENTITIES,                                                        \
    TUD_AUDIO_DESC_STD_AS_INT(                                                \
        /*_itfnum*/ ITF_NUM_AUDIO_STREAMING,                                  \
        /*_altset*/ AS_ALT_ZERO_BW,                                           \
        /*_nEPs*/   0,                                                        \
        /*_stridx*/ 0x00),                                                    \
    AUDIO_AS_STREAM_ALT(AS_ALT_16BIT, AUDIO_16BIT_SUBSLOT, AUDIO_16BIT_RESOLUTION, _fbsize), \
    AUDIO_AS_STREAM_ALT(AS_ALT_24BIT, AUDIO_24BIT_SUBSLOT, AUDIO_24BIT_RESOLUTION, _fbsize)

TU_VERIFY_STATIC(sizeof((uint8_t const[]){ AUDIO_AC_ENTITIES }) == USB2SPDIF_AC_ENTITIES_LEN,
                 "CS AC wTotalLength does not match the entity descriptors");
TU_VERIFY_STATIC(sizeof((uint8_t const[]){ AUDIO_AS_STREAM_ALT(AS_ALT_24BIT, AUDIO_24BIT_SUBSLOT,
                                                               AUDIO_24BIT_RESOLUTION, AUDIO_FB_EP_SIZE) })
                     == USB2SPDIF_AS_ALT_STREAM_DESC_LEN,
                 "AS alt setting length mismatch");
TU_VERIFY_STATIC(sizeof((uint8_t const[]){ USB2SPDIF_AUDIO_FUNCTION_DESCRIPTOR(AUDIO_FB_EP_SIZE) })
                     == USB2SPDIF_AUDIO_DESC_LEN,
                 "USB2SPDIF_AUDIO_DESC_LEN does not match the audio descriptor block");
TU_VERIFY_STATIC(USB2SPDIF_AUDIO_DESC_LEN == 178, "unexpected audio block length");

//--------------------------------------------------------------------+
// Configuration descriptor
//--------------------------------------------------------------------+

#define CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + USB2SPDIF_AUDIO_DESC_LEN + TUD_RPI_RESET_DESC_LEN)

#define CONFIG_ATTR_BUS_POWERED 0x00
#define CONFIG_MAX_POWER_MA     100

#define CONFIGURATION_DESCRIPTOR(_fbsize)                                     \
    TUD_CONFIG_DESCRIPTOR(                                                    \
        /*config_num*/ 1,                                                     \
        /*_itfcount*/  ITF_NUM_TOTAL,                                         \
        /*_stridx*/    0x00,                                                  \
        /*_total_len*/ CONFIG_TOTAL_LEN,                                      \
        /*_attribute*/ CONFIG_ATTR_BUS_POWERED,                               \
        /*_power_ma*/  CONFIG_MAX_POWER_MA),                                  \
    USB2SPDIF_AUDIO_FUNCTION_DESCRIPTOR(_fbsize),                             \
    TUD_RPI_RESET_DESCRIPTOR(ITF_NUM_RESET, STRID_RESET)

static uint8_t const desc_configuration[] = {
    CONFIGURATION_DESCRIPTOR(AUDIO_FB_EP_SIZE)
};

static uint8_t const desc_configuration_apple[] = {
    CONFIGURATION_DESCRIPTOR(AUDIO_FB_EP_SIZE_APPLE)
};

TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
                 "configuration descriptor length mismatch");
TU_VERIFY_STATIC(sizeof(desc_configuration_apple) == CONFIG_TOTAL_LEN,
                 "Apple configuration descriptor length mismatch");
TU_VERIFY_STATIC(CONFIG_TOTAL_LEN == 196, "unexpected configuration length");

//--------------------------------------------------------------------+
// Host detection
//--------------------------------------------------------------------+

// Apple hosts request a string descriptor right after the device
// descriptor; Windows and Linux request the configuration (or BOS) first.
// Based on TinyUSB's quirk_os_guessing example; the SDK owns the BOS
// callback, so only string and configuration requests are observed.
typedef enum {
    HOST_UNKNOWN = 0,
    HOST_APPLE,
    HOST_OTHER,
} host_guess_t;

static host_guess_t host_guess;

// The first string or configuration request after the device descriptor
// decides.
static void host_note_request(host_guess_t if_first)
{
    if (!host_decided) {
        host_guess = if_first;
        host_decided = true;
    }
}

bool usb_host_is_apple(void)
{
    return host_guess == HOST_APPLE;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    host_note_request(HOST_OTHER);
    return usb_host_is_apple() ? desc_configuration_apple : desc_configuration;
}

//--------------------------------------------------------------------+
// String descriptors
//--------------------------------------------------------------------+

// Fits one 64-byte EP0 packet with the header.
#define STRING_MAX_CHARS        31

#define USB_LANGID_EN_US        0x0409

// Unique board ID as hex, so several bridges get distinct instances.
static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

// ASCII only; each byte becomes one UTF-16 code unit.
static char const *const string_table[STRID_COUNT] = {
    [STRID_LANGID]         = NULL,
    [STRID_MANUFACTURER]   = USB_MANUFACTURER_STR,
    [STRID_PRODUCT]        = USB_PRODUCT_STR,
    [STRID_SERIAL]         = serial_str,        // filled on first request
    [STRID_AUDIO_FUNCTION] = "USB to S/PDIF",
    [STRID_RESET]          = "Reset",
};

TU_VERIFY_STATIC(sizeof(USB_MANUFACTURER_STR) - 1 <= STRING_MAX_CHARS, "manufacturer string too long");
TU_VERIFY_STATIC(sizeof(USB_PRODUCT_STR) - 1 <= STRING_MAX_CHARS, "product string too long");
TU_VERIFY_STATIC(sizeof(serial_str) - 1 <= STRING_MAX_CHARS, "serial string too long");

// Static: TinyUSB sends it after the callback returns.
static uint16_t desc_str[1 + STRING_MAX_CHARS];

// 126 characters is the USB string maximum.
#define DIAG_STRING_MAX_CHARS   126
static uint16_t diag_desc_str[1 + DIAG_STRING_MAX_CHARS];

static uint16_t const *diag_string_desc(void)
{
    char const *str = diag_string();
    size_t n = 0;
    while (n < DIAG_STRING_MAX_CHARS && str[n] != '\0') {
        diag_desc_str[1 + n] = (uint8_t)str[n];
        n++;
    }
    diag_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return diag_desc_str;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    size_t chr_count;

    host_note_request(HOST_APPLE);

    if (index == STRID_DIAG) {
        return diag_string_desc();
    }

    if (index == STRID_LANGID) {
        desc_str[1] = USB_LANGID_EN_US;
        chr_count = 1;
    } else {
        // Unknown indices, including 0xEE (MS OS 1.0), are STALLed.
        if (index >= STRID_COUNT || string_table[index] == NULL) {
            return NULL;
        }

        if (index == STRID_SERIAL && serial_str[0] == '\0') {
            pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
        }

        char const *str = string_table[index];
        chr_count = 0;
        while (chr_count < STRING_MAX_CHARS && str[chr_count] != '\0') {
            desc_str[1 + chr_count] = (uint8_t)str[chr_count];
            chr_count++;
        }
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
