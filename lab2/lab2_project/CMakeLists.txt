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
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(lab2_project C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()



add_executable(lab2_project rotary_encoder.c VGA/vga16_graphics_v3.c)

pico_generate_pio_header(lab2_project ${CMAKE_CURRENT_LIST_DIR}/VGA/hsync.pio)
pico_generate_pio_header(lab2_project ${CMAKE_CURRENT_LIST_DIR}/VGA/vsync.pio)
pico_generate_pio_header(lab2_project ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb.pio)



# Add executable. Default name is the project name, version 0.1



pico_set_program_name(lab2_project "lab2_project")
pico_set_program_version(lab2_project "0.1")

# Modify the below lines to enable/disable output over UART/USB
# pico_enable_stdio_uart(lab2_project 0)
# pico_enable_stdio_usb(lab2_project 0)




# Add the standard library to the build
target_link_libraries(lab2_project
        pico_stdlib
        hardware_gpio
        pico_divider
        pico_multicore
        pico_sync
        hardware_pio 
        hardware_dma 
        hardware_adc 
        hardware_sync
        hardware_irq
        pico_multicore
        hardware_clocks
        hardware_vreg
        hardware_pll
        hardware_spi
        pico_bootsel_via_double_reset 
        )

# Add the standard include files to the build
target_include_directories(lab2_project PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(lab2_project)

