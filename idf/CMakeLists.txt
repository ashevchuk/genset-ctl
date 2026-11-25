cmake_minimum_required(VERSION 3.16)

# Project root (one level up) is itself a component (its own CMakeLists.txt,
# wraps this project's device-driver .cpp/.h files) — read directly, no
# copies. See ../CMakeLists.txt for why this coexists with arduino-builder.
# ../libraries holds frozen local copies of the third-party libraries this
# firmware links.
set(EXTRA_COMPONENT_DIRS ".." "../libraries")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-gen-ctl-ai-rtos-idf)
