cmake_minimum_required(VERSION 3.13)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)

project(laser_distance C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# FreeRTOS
set(FREERTOS_KERNEL_PATH "C:/Users/Rafael/Desktop/embarcatech/Nova_pasta/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

add_executable(laser_distance
    laser_distance.c
    mpu6050/mpu6050.c
    ssd1306.c
    display.c
)

pico_generate_pio_header(laser_distance ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_include_directories(laser_distance PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # FreeRTOSConfig.h + lwipopts.h aqui
    ${CMAKE_CURRENT_LIST_DIR}/mpu6050
)

pico_set_program_name(laser_distance "laser_distance")
pico_set_program_version(laser_distance "1.0")
pico_enable_stdio_usb(laser_distance 1)
pico_enable_stdio_uart(laser_distance 0)

target_link_libraries(laser_distance
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    FreeRTOS-Kernel-Heap4
    pico_fix_rp2040_usb_device_enumeration
    hardware_i2c
    hardware_pwm
    hardware_pio
    m
)

pico_add_extra_outputs(laser_distance)