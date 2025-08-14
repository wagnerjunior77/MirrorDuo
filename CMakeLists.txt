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
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT (VS Code Pico) ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# =================================

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(MirrorDuo C CXX ASM)
pico_sdk_init()

# -----------------------------
# Alvo 1: BitDog-A (hello)
# -----------------------------
add_executable(mirrorbox_hello_a
    mirrorbox_hello_a.c
)
pico_enable_stdio_usb(mirrorbox_hello_a 1)
pico_enable_stdio_uart(mirrorbox_hello_a 1)
target_link_libraries(mirrorbox_hello_a
    pico_stdlib
    pico_cyw43_arch_none       # controla LED da Pico W sem rede
    hardware_i2c
    hardware_gpio
    hardware_irq
)
pico_set_program_name(mirrorbox_hello_a "mirrorbox_hello_a")
pico_set_program_version(mirrorbox_hello_a "0.1")
pico_add_extra_outputs(mirrorbox_hello_a)

# -----------------------------
# Alvo 2: BitDog-B (hello)
# -----------------------------
add_executable(therapad_hello_b
    therapad_hello_b.c
)
pico_enable_stdio_usb(therapad_hello_b 1)
pico_enable_stdio_uart(therapad_hello_b 1)
target_link_libraries(therapad_hello_b
    pico_stdlib
    pico_cyw43_arch_none       # controla LED da Pico W sem rede
    hardware_i2c
    hardware_gpio
    hardware_irq
)
pico_set_program_name(therapad_hello_b "therapad_hello_b")
pico_set_program_version(therapad_hello_b "0.1")
pico_add_extra_outputs(therapad_hello_b)
