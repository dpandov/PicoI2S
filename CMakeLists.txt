# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Target board. Pico 2 = RP2350, ARM Cortex-M33 cores, secure mode.
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")

# TinyUSB from the lib/tinyusb submodule (pinned release) instead of the
# version bundled with the SDK.
set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb)
if(NOT EXISTS ${PICO_TINYUSB_PATH}/src/tusb.h)
    message(FATAL_ERROR "lib/tinyusb is empty: run 'git submodule update --init'")
endif()

# Pull in the Pico SDK (must happen before project()).
include(pico_sdk_import.cmake)

project(PicoI2S C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(PicoI2S
    src/main.c
    src/app/audio_pipeline.c
    src/app/clock_servo.c
    src/app/log.c
    src/app/monitor.c
    src/app/status_led.c
    src/app/test_tone.c
    src/clock/audio_clock.c
    src/diag/diag.c
    src/i2s/i2s_tx.c
    src/spdif/spdif_encode.c
    src/spdif/spdif_rx.c
    src/spdif/spdif_rx_parse.c
    src/spdif/spdif_tx.c
    src/usb/usb_audio.c
    src/usb/usb_descriptors.c
)

pico_generate_pio_header(PicoI2S ${CMAKE_CURRENT_LIST_DIR}/src/spdif/spdif_tx.pio)
pico_generate_pio_header(PicoI2S ${CMAKE_CURRENT_LIST_DIR}/src/spdif/spdif_pacer.pio)
pico_generate_pio_header(PicoI2S ${CMAKE_CURRENT_LIST_DIR}/src/spdif/spdif_rx.pio)

# Includes are written relative to src/ (e.g. "usb/usb_audio.h").
# TinyUSB includes tusb_config.h by bare name, hence src/usb.
target_include_directories(PicoI2S PRIVATE src src/usb)

# Board wiring (src/board_config.h): the Waveshare RP2350-One by default, or
# the Raspberry Pi Pico 2. Both carry the RP2350A with 4 MB of flash and a
# 12 MHz crystal, so PICO_BOARD stays pico2 for both.
set(PICOI2S_BOARD "rp2350-one" CACHE STRING "Board wiring: rp2350-one or pico2")
set_property(CACHE PICOI2S_BOARD PROPERTY STRINGS rp2350-one pico2)
if(PICOI2S_BOARD STREQUAL "rp2350-one")
    target_compile_definitions(PicoI2S PRIVATE PICOI2S_BOARD_RP2350_ONE=1)
    pico_generate_pio_header(PicoI2S ${CMAKE_CURRENT_LIST_DIR}/src/app/ws2812.pio)
elseif(PICOI2S_BOARD STREQUAL "pico2")
    target_compile_definitions(PicoI2S PRIVATE PICOI2S_BOARD_PICO2=1)
else()
    message(FATAL_ERROR "PICOI2S_BOARD must be rp2350-one or pico2")
endif()

# Outputs a 1 kHz, -18 dBFS test tone at 48 kHz from boot, without a host,
# to check the wiring and receiver lock.
option(PICOI2S_TEST_TONE "Output a 1 kHz test tone on S/PDIF" OFF)
if(PICOI2S_TEST_TONE)
    target_compile_definitions(PicoI2S PRIVATE PICOI2S_TEST_TONE=1)
endif()

# Format jumper code 010: right-justified 24-bit instead of 16-bit.
option(PICOI2S_RJ24 "Right-justified format with 24-bit words instead of 16" OFF)
if(PICOI2S_RJ24)
    target_compile_definitions(PicoI2S PRIVATE PICOI2S_RJ24=1)
endif()

# Decodes the S/PDIF and I2S outputs on core 1 and reports them as USB
# strings, for tools/selftest.py (src/selftest/selftest.h). With the test
# tone, only framing and channel status are checked.
option(PICOI2S_SELFTEST "Check the S/PDIF and I2S outputs on core 1" OFF)
if(PICOI2S_SELFTEST)
    target_sources(PicoI2S PRIVATE
        src/selftest/selftest.c
        src/selftest/selftest_decode.c
        src/selftest/selftest_i2s_decode.c
        src/selftest/selftest_i2s_expect.c
        src/selftest/selftest_report.c
    )
    pico_generate_pio_header(PicoI2S ${CMAKE_CURRENT_LIST_DIR}/src/selftest/selftest_capture.pio)
    target_compile_definitions(PicoI2S PRIVATE PICOI2S_SELFTEST=1)
endif()

# Core 1 runs the I2S oversampling filter (and the self-test decoders).
target_link_libraries(PicoI2S
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_clocks
    hardware_watchdog
    hardware_pio
    hardware_dma
    tinyusb_device
    pico_usb_reset
)

# picotool reset interface on custom descriptors (reflash without BOOTSEL).
# The library also provides the BOS / MS OS 2.0 descriptors for WinUSB.
target_compile_definitions(PicoI2S PRIVATE
    PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE=1
    PICO_USB_RESET_SUPPORT_MS_OS_20_DESCRIPTOR=1
    PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF=2      # must equal ITF_NUM_RESET
    PICO_USB_RESET_INCLUDE_DEFAULT_APP_DRIVER_CB=1
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=0
)

# Feedback format for Apple hosts (src/usb/usb_audio.c).
target_link_options(PicoI2S PRIVATE "LINKER:--wrap=usbd_edpt_xfer")

# panic() is recorded by the diagnostics module (src/diag/diag.h).
target_compile_definitions(PicoI2S PRIVATE PICO_PANIC_FUNCTION=diag_panic)
# TinyUSB's USB interrupt path in RAM: isochronous transfers are handled there.
target_compile_definitions(PicoI2S PRIVATE PICO_RP2040_USB_FAST_IRQ=1)

# USB is the audio device, so the console is on UART0.
pico_enable_stdio_uart(PicoI2S 0)
pico_enable_stdio_usb(PicoI2S 0)

pico_add_extra_outputs(PicoI2S)
