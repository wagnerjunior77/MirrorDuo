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

# ---------- libs ----------
# OLED (usa I2C)
add_library(ssd1306 STATIC
    src/ssd1306_i2c.c
)
target_link_libraries(ssd1306
    pico_stdlib
    hardware_i2c
)
# headers são incluídos como "src/ssd1306*.h"
target_include_directories(ssd1306 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Sensor de cor (TCS34725) como biblioteca reutilizável
add_library(corlib STATIC
    src/cor.c
)
target_link_libraries(corlib
    pico_stdlib
    hardware_i2c
)
# headers são incluídos como "src/cor.h"
target_include_directories(corlib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# ---------- executável principal (orquestrador) ----------
add_executable(main
    main.c
)

target_link_libraries(main
    pico_stdlib
    hardware_i2c
    ssd1306
    corlib
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")
pico_add_extra_outputs(main)
