cmake_minimum_required(VERSION 3.20.0)

set(BOARD rpi_pico)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app LANGUAGES C)

# include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

# target_sources(app PRIVATE src/main.c)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# add_subdirectory(src)
# include_directories(include)
