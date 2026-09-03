# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(lab1_new C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(lab1_new integrated.c)

#target_sources(lab1_new PRIVATE integrated.c)

# Add the standard library to the build
target_link_libraries( lab1_new
        pico_stdlib
        pico_stdlib 
        pico_sync
        hardware_sync
        hardware_clocks
        hardware_vreg
        hardware_clocks
        hardware_adc
        pico_bootsel_via_double_reset 
        hardware_spi 
        hardware_timer 
        hardware_irq
        )
#pico_add_extra_outputs(Audio_Timer_Interrupt_DDS)
# create map/bin/hex file etc.
#pico_add_extra_outputs(simple_adc)







