# COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
#
# Author: Marko Sagadin <marko@irnas.eu>

cmake_minimum_required(VERSION 3.20.0)

# create compile_commands.json for clang
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(basic)

target_sources(app PRIVATE src/main.c)
