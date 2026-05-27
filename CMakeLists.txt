# Native Linux build for microReticulum_Firmware.
#
# This file is for the native target only. Embedded boards (ESP32/nRF52)
# continue to build via platformio.ini.
#
# Quickstart:
#   cmake -B build
#   cmake --build build -j
#   ./build/microreticulum --config microreticulum.conf --data-dir ./state
#
# Dependencies (system packages):
#   - libgpiod-dev   (>= 2.0; for /dev/gpiochip edge events)
#   - pkg-config
#   - A C++17 toolchain (g++ >= 8, clang >= 9)
#
# All C/C++ dependencies (Portduino, microReticulum, microStore, Crypto)
# are pulled in via FetchContent so the host doesn't need any pre-installed
# Arduino library.

cmake_minimum_required(VERSION 3.16)
project(microreticulum CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)  # we use -std=gnu++17 to match platformio.ini

include(FetchContent)

# --- Dependencies ---------------------------------------------------------

# Portduino: Arduino-API shim over /dev/spidev + libgpiod + termios.
# Used by the existing modem drivers (sx126x/sx127x/sx128x) without
# any modification.
FetchContent_Declare(
  portduino
  GIT_REPOSITORY https://github.com/meshtastic/framework-portduino.git
  GIT_TAG        master  # pin to a known-good SHA before release
)

# microReticulum: the C++ RNS stack. Declares native portability via
# library.json (platforms: "*", frameworks: "*") and uses POSIX clocks
# under #ifdef ARDUINO guards.
FetchContent_Declare(
  microReticulum
  GIT_REPOSITORY https://github.com/attermann/microReticulum.git
  GIT_TAG        master
)

# microStore: persistent key/value adapter. Provides PosixFileSystem which
# we register at startup so RNS persists to the working directory.
FetchContent_Declare(
  microStore
  GIT_REPOSITORY https://github.com/attermann/microStore.git
  GIT_TAG        master
)

# Crypto: native-buildable port of Arduino Crypto used by RNS Identity.
FetchContent_Declare(
  Crypto
  GIT_REPOSITORY https://github.com/attermann/Crypto.git
  GIT_TAG        master
)

FetchContent_MakeAvailable(portduino microReticulum microStore Crypto)

# libgpiod via pkg-config (Portduino's edge events need >= 2.0).
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# --- Sources --------------------------------------------------------------

# The Arduino sketch entrypoint is RNode_Firmware.ino. CMake doesn't compile
# .ino files by default; we point at it as C++.
set_source_files_properties(${CMAKE_SOURCE_DIR}/RNode_Firmware.ino
                            PROPERTIES LANGUAGE CXX)

set(MR_NATIVE_SOURCES
  ${CMAKE_SOURCE_DIR}/RNode_Firmware.ino
  ${CMAKE_SOURCE_DIR}/sx126x.cpp
  ${CMAKE_SOURCE_DIR}/sx127x.cpp
  ${CMAKE_SOURCE_DIR}/sx128x.cpp
  ${CMAKE_SOURCE_DIR}/native/main.cpp
  ${CMAKE_SOURCE_DIR}/native/config.cpp
  ${CMAKE_SOURCE_DIR}/native/PinMap.cpp
)

add_executable(microreticulum ${MR_NATIVE_SOURCES})

target_include_directories(microreticulum
  PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/native
    ${GPIOD_INCLUDE_DIRS}
)

# Compile-time options. The MODEM default is SX1262 (most common SPI-attached
# modem on RPi HATs and FemtoFox). Override on the command line:
#   cmake -DMR_MODEM=SX1276 ...
set(MR_MODEM "SX1262" CACHE STRING "LoRa modem variant: SX1262 / SX1276 / SX1278 / SX1280")
set(MR_HAS_DISPLAY OFF CACHE BOOL "Enable OLED display support (unused on native)")

target_compile_definitions(microreticulum
  PRIVATE
    MCU_VARIANT=MCU_NATIVE
    BOARD_MODEL=BOARD_NATIVE_LINUX
    MODEM=${MR_MODEM}
    HAS_RNS
    RNS_USE_FS
    RNS_PERSIST_PATHS
    USTORE_USE_POSIXFS=1
    URTN_PATH_TABLE_MAX_RECS=2000
    RNS_PATH_TABLE_SEGMENT_SIZE=98304   # 96 KB; native has plenty of RAM
    RNS_PATH_TABLE_SEGMENT_COUNT=8
    MSGPACK_USE_BOOST=OFF
)

target_compile_options(microreticulum
  PRIVATE
    -Wall
    -Wno-missing-field-initializers
    -Wno-format
    -fexceptions
)

target_link_libraries(microreticulum
  PRIVATE
    portduino
    microReticulum
    microStore
    Crypto
    ${GPIOD_LIBRARIES}
    pthread
    rt
)

# --- Install --------------------------------------------------------------

install(TARGETS microreticulum RUNTIME DESTINATION bin)
