cmake_minimum_required(VERSION 3.12)
#set(PICO_BOARD tzt_rgb_usbc_rp2040)
set(CFG_TUSB_OS OPT_OS_FREERTOS)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
set(TARGET dev_usbbridge_jtag)

project(TUsbBenchmark C CXX ASM)

include(PicoSDKConfig.cmake)

# Pull in FreeRTOS
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040 BINARY_DIR)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.13)

add_executable(dev_usbbridge_jtag)

set(esp_loader_srcs
    components/esp_loader/src/esp_loader.c
    components/esp_loader/src/esp_targets.c
    components/esp_loader/src/serial_comm.c
    components/esp_loader/src/md5_hash.c
    components/esp_loader/port/rp2040_port.c
)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dev_usbbridge_jtag ${CMAKE_CURRENT_LIST_DIR}/ws2812/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
pico_generate_pio_header(dev_usbbridge_jtag ${CMAKE_CURRENT_LIST_DIR}/jtag.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(dev_usbbridge_jtag PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        ${CMAKE_CURRENT_LIST_DIR}/freertos_hooks.c
        ${CMAKE_CURRENT_LIST_DIR}/ws2812/ws2812.c
        ${CMAKE_CURRENT_LIST_DIR}/jtag.c
        ${CMAKE_CURRENT_LIST_DIR}/serial.c
        ${CMAKE_CURRENT_LIST_DIR}/msc.c
        ${esp_loader_srcs}
       )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(dev_usbbridge_jtag PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/ws2812
        ${CMAKE_CURRENT_LIST_DIR}/generated
        ${CMAKE_CURRENT_LIST_DIR}/components/esp_loader/include
        ${CMAKE_CURRENT_LIST_DIR}/components/esp_loader/private_include
         )

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(dev_usbbridge_jtag PUBLIC pico_stdlib pico_stdio hardware_pio hardware_dma tinyusb_device tinyusb_board FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 pico_unique_id)

pico_add_extra_outputs(dev_usbbridge_jtag)
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND arm-none-eabi-size --format=gnu "${TARGET}.elf")

# add url via pico_set_program_url
# example_auto_set_url(dev_usbbridge_jtag)
