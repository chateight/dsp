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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
#set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
#if (EXISTS ${picoVscode})
#    include(${picoVscode})
#endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

set(PICO_SDK_PATH "$ENV{HOME}/pi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(dsp C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Create a library for the LCD driver
add_library(lcd_driver STATIC
    lcd_st7789_library.c
    font_5x7.c
)

add_library(CMSISDSP STATIC IMPORTED GLOBAL)
set_target_properties(CMSISDSP PROPERTIES IMPORTED_LOCATION
  $ENV{HOME}/pi/pico/CMSISDSP/build/bin_dsp/libCMSISDSP.a)

#target_link_libraries(dsp CMSISDSP ...)

# Add executable. Default name is the project name, version 0.1

add_executable(dsp dsp.c )

pico_set_program_name(dsp "dsp")
pico_set_program_version(dsp "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(dsp 0)
pico_enable_stdio_usb(dsp 1)

# Add the standard library to the build
target_link_libraries(dsp
        pico_stdlib
        pico_multicore
        hardware_adc
        hardware_dma
        hardware_pio
        hardware_pwm
        hardware_spi
        CMSISDSP
        lcd_driver
        hardware_timer 
        pico_multicore
        pico_cyw43_arch_none
        hardware_i2c
)

target_link_libraries(lcd_driver
    pico_stdlib
    hardware_spi
)

# Add the standard include files to the build
target_include_directories(dsp PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        $ENV{HOME}/pi/pico/CMSISDSP/CMSIS-DSP/Include
        $ENV{HOME}/pi/pico/CMSISDSP/CMSIS_6/CMSIS/Core/Include
        ${PICO_SDK_PATH}/src/rp2
)

target_include_directories(lcd_driver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/src/common
    ${PICO_SDK_PATH}/src/rp2
)

pico_add_extra_outputs(dsp)

