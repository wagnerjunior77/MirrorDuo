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

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(MirrorDuo C CXX ASM)
pico_sdk_init()

# -----------------------------
# Alvo 1: BitDog-A (Oxímetro)
# -----------------------------
add_executable(oximetro
    oximetro.c
)
pico_enable_stdio_usb(oximetro 1)
pico_enable_stdio_uart(oximetro 1)
target_link_libraries(oximetro
    pico_stdlib
    pico_cyw43_arch_none
    hardware_i2c
    hardware_gpio
    hardware_irq
    m
)
pico_set_program_name(oximetro "oximetro")
pico_set_program_version(oximetro "0.1")
pico_add_extra_outputs(oximetro)

# -----------------------------
# Alvo 2: BitDog-B (Sensor de cor)
# -----------------------------
add_executable(cor
    cor.c
)
pico_enable_stdio_usb(cor 1)
pico_enable_stdio_uart(cor 1)
target_link_libraries(cor
    pico_stdlib
    pico_cyw43_arch_none
    hardware_i2c
    hardware_gpio
    hardware_irq
    m
)
pico_set_program_name(cor "cor")
pico_set_program_version(cor "0.1")
pico_add_extra_outputs(cor)
