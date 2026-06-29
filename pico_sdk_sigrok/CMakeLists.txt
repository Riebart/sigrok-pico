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
#See https://github.com/pico-coder/sigrok-pico/pull/63/  
#setting this allows the DTR to be ignored.  In addition to this, the use of 
#tud_cdc_connected should be replaced with tud_ready.
add_definitions(-DPICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
pico_sdk_init()

add_executable(pico_sdk_sigrok
  pico_sdk_sigrok.c
  sr_device.c
)

# ---------------------------------------------------------------------------
# PICO_MODE — selects the channel/sample-rate configuration:
#   0  baseline: 21 digital + 3 analog  (default)
#   1  digital-26: 26 digital channels
#   2  digital-32: 32 digital channels
#
# Pass from the cmake command line as:   -DPICO_MODE=2
# The CI workflows inject this as a CMake cache variable; it must be
# forwarded here to the C preprocessor so that #if PICO_MODE == N
# conditionals in the source files are evaluated correctly.
# ---------------------------------------------------------------------------
if(NOT DEFINED PICO_MODE)
    set(PICO_MODE 0 CACHE STRING "Sigrok-pico channel mode (0=baseline, 1=dig26, 2=dig32)")
endif()
message(STATUS "Building with PICO_MODE=${PICO_MODE}")
target_compile_definitions(pico_sdk_sigrok PUBLIC PICO_MODE=${PICO_MODE})

pico_enable_stdio_usb(pico_sdk_sigrok 1)
pico_enable_stdio_uart(pico_sdk_sigrok 0)

pico_add_extra_outputs(pico_sdk_sigrok)

target_link_libraries(
    pico_sdk_sigrok 
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_pio
    hardware_sync
    hardware_spi
#    hardware_sio
#    hardware_gpio
#    hardware_timer
    pico_multicore
    pico_stdio_usb
)
