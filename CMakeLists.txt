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

# Pull in the Pico SDK (must happen before project()).
include(pico_sdk_import.cmake)

project(usb2spdif C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(usb2spdif
    src/main.c
    src/app/audio_pipeline.c
    src/app/log.c
    src/app/monitor.c
    src/app/status_led.c
    src/app/test_tone.c
    src/clock/audio_clock.c
    src/diag/diag.c
    src/spdif/spdif_encode.c
    src/spdif/spdif_tx.c
    src/usb/usb_audio.c
    src/usb/usb_descriptors.c
)

pico_generate_pio_header(usb2spdif ${CMAKE_CURRENT_LIST_DIR}/src/spdif/spdif_tx.pio)

# Includes are written relative to src/ (e.g. "usb/usb_audio.h").
# TinyUSB includes tusb_config.h by bare name, hence src/usb.
target_include_directories(usb2spdif PRIVATE src src/usb)

# Outputs a 1 kHz, -18 dBFS test tone at 48 kHz from boot, without a host,
# to check the wiring and receiver lock.
option(USB2SPDIF_TEST_TONE "Output a 1 kHz test tone on S/PDIF" OFF)
if(USB2SPDIF_TEST_TONE)
    target_compile_definitions(usb2spdif PRIVATE USB2SPDIF_TEST_TONE=1)
endif()

# Decodes the S/PDIF output on core 1 and reports it as a USB string, for
# tools/selftest.py (src/selftest/selftest.h). With the test tone, only
# framing and channel status are checked.
option(USB2SPDIF_SELFTEST "Check the S/PDIF output on core 1" OFF)
if(USB2SPDIF_SELFTEST)
    target_sources(usb2spdif PRIVATE
        src/selftest/selftest.c
        src/selftest/selftest_decode.c
        src/selftest/selftest_report.c
    )
    pico_generate_pio_header(usb2spdif ${CMAKE_CURRENT_LIST_DIR}/src/selftest/selftest_capture.pio)
    target_compile_definitions(usb2spdif PRIVATE USB2SPDIF_SELFTEST=1)
    target_link_libraries(usb2spdif pico_multicore)
endif()

target_link_libraries(usb2spdif
    pico_stdlib
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
target_compile_definitions(usb2spdif PRIVATE
    PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE=1
    PICO_USB_RESET_SUPPORT_MS_OS_20_DESCRIPTOR=1
    PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF=2      # must equal ITF_NUM_RESET
    PICO_USB_RESET_INCLUDE_DEFAULT_APP_DRIVER_CB=1
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=0
)

# panic() is recorded by the diagnostics module (src/diag/diag.h).
target_compile_definitions(usb2spdif PRIVATE PICO_PANIC_FUNCTION=diag_panic)

# USB is the audio device, so the console is on UART0.
pico_enable_stdio_uart(usb2spdif 0)
pico_enable_stdio_usb(usb2spdif 0)

pico_add_extra_outputs(usb2spdif)
