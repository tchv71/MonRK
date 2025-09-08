# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
add_compile_options(
    -DPICO_STDIO_ENABLE_CRLF_SUPPORT=0
    -DPICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
    -DCFG_TUSB_CONFIG_FILE=\"./tusb_config2.h\"
    #-DPICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR=0
)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
#set(PROGRAM MonRK)

project(MonRk C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


add_subdirectory(WIZnet-PICO-C)

# Add executable. Default name is the project name, version 0.1

add_executable(MonRk )
target_sources(MonRk PRIVATE main.cpp
# Serial.cpp RP2040USB.cpp
 loop.cpp sd.c 86rksd.c fs.c proto.c WIZnet-PICO-C/port/ioLibrary_Driver/src/w5x00_spi.c usb_msc_driver.c
 cpmfs.c device.c
)

pico_set_program_name(MonRk "MonRK")
pico_set_program_version(MonRk "0.1")

# generate header file from pio
pico_generate_pio_header(MonRk ${CMAKE_CURRENT_LIST_DIR}/fifo.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(MonRk 0)
pico_enable_stdio_usb(MonRk 1)

# Add the standard library to the build
target_link_libraries(MonRk PRIVATE
        pico_stdlib
        hardware_pio
        hardware_spi
        hardware_dma
        hardware_rtc
        #tinyusb_device
        pico_multicore
        pico_unique_id
        IOLIBRARY_FILES
        ETHERNET_FILES
        SNTP_FILES
        FTPSERVER_FILES
        TIMER_FILES
        )

# Add the standard include files to the build
target_include_directories(MonRk PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(MonRk)

