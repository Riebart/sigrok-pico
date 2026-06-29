# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_sdk_sigrok C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(pico_sdk_sigrok PRIVATE ADS1256_ENABLE_STDIO_UART=1)
endif()

# ADS1256 compile-time overrides (PICO_MODE 3 only).
# Pass on the cmake command line, e.g.:
#   cmake -DADS1256_PGA_GAIN=2 -DADS1256_BUF_ENABLE=1 ..
# Defaults are defined in sr_device.h when not specified here.
if(DEFINED ADS1256_PGA_GAIN)
    add_compile_definitions(ADS1256_PGA_GAIN=${ADS1256_PGA_GAIN})
endif()
if(DEFINED ADS1256_BUF_ENABLE)
    add_compile_definitions(ADS1256_BUF_ENABLE=${ADS1256_BUF_ENABLE})
endif()
if(DEFINED ADS1256_CLKIN_HZ)
    add_compile_definitions(ADS1256_CLKIN_HZ=${ADS1256_CLKIN_HZ})
endif()
if(DEFINED ADS1256_DRATE_REG)
    add_compile_definitions(ADS1256_DRATE_REG=${ADS1256_DRATE_REG})
endif()

#See https://github.com/pico-coder/sigrok-pico/pull/63/
#setting this allows the DTR to be ignored.  In addition to this, the use of
#tud_cdc_connected should be replaced with tud_ready.
add_definitions(-DPICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
pico_sdk_init()

add_executable(pico_sdk_sigrok
  pico_sdk_sigrok.c
  sr_device.c
  ads1256.c
)

# ---------------------------------------------------------------------------
# PICO_MODE — selects the channel/sample-rate configuration:
#   0  baseline: 21 digital + 3 analog  (default)
#   1  digital-26: 26 digital channels
#   2  digital-32: 32 digital channels
#   3  Uses an external ADC for acquisition
#
# Pass from the cmake command line as:   -DPICO_MODE=2
# The CI workflows inject this as a CMake cache variable; it must be
# forwarded here to the C preprocessor so that #if PICO_MODE == N
# conditionals in the source files are evaluated correctly.
# ---------------------------------------------------------------------------
if(NOT DEFINED PICO_MODE)
    set(PICO_MODE 0 CACHE STRING "Sigrok-pico channel mode (0=baseline, 1=dig26, 2=dig32, 3=external-adc)")
endif()
message(STATUS "Building with PICO_MODE=${PICO_MODE}")
target_compile_definitions(pico_sdk_sigrok PUBLIC PICO_MODE=${PICO_MODE})

if(DEFINED ADS1256_ENABLE_STDIO_UART AND ADS1256_ENABLE_STDIO_UART)
    message(STATUS "ADS1256_ENABLE_STDIO_UART: stdio routed to UART1 on GP8/GP9")
    add_compile_definitions(ADS1256_ENABLE_STDIO_UART=1)

    pico_enable_stdio_usb(pico_sdk_sigrok 0)
    pico_enable_stdio_uart(pico_sdk_sigrok 1)

    target_compile_definitions(pico_sdk_sigrok PRIVATE
        PICO_DEFAULT_UART=0
        PICO_DEFAULT_UART_TX_PIN=12
        PICO_DEFAULT_UART_RX_PIN=13
    )
else()
    pico_enable_stdio_usb(pico_sdk_sigrok 1)
    pico_enable_stdio_uart(pico_sdk_sigrok 0)
endif()

pico_add_extra_outputs(pico_sdk_sigrok)

target_link_libraries(
    pico_sdk_sigrok
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_pio
    hardware_spi
    hardware_sync
    hardware_spi
#    hardware_sio
#    hardware_gpio
#    hardware_timer
    pico_multicore
    pico_stdio_usb
)
