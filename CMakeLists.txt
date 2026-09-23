cmake_minimum_required(VERSION 3.24)
project(arduino_rpc CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Warning flags for our own targets only (third-party targets keep theirs).
option(SERIAL_RPC_WERROR "Treat warnings in our own code as errors" OFF)
function(serial_rpc_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8
                                             /Zc:__cplusplus)
    if(SERIAL_RPC_WERROR)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(SERIAL_RPC_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# Shared codec + (later) framing/server/host-RPC headers, available to both
# the host build and, via arduino-cli --library, the Arduino build.
add_library(serial_rpc INTERFACE)
target_include_directories(serial_rpc INTERFACE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/extras/host/include
)
target_compile_features(serial_rpc INTERFACE cxx_std_20)

include(FetchContent)
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.11
)
# doctest 2.4.11's own CMakeLists.txt declares cmake_minimum_required(3.0),
# which current CMake (>= 4.0) refuses to configure at all. Tell FetchContent
# to evaluate that subproject under the CMake 3.5 policy set instead of
# failing; this does not affect policies for our own CMakeLists.txt.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
FetchContent_MakeAvailable(doctest)
unset(CMAKE_POLICY_VERSION_MINIMUM)

# -----------------------------------------------------------------------------
# Dependencies for extras/host/tools/rpc_repl (docs/PLAN.md component 7)
# only. None of these are linked into `serial_rpc`, `serialport_smoke`, or
# the serial_rpc_tests* targets below -- only `rpc_repl` and `rpc_repl_tests`
# use them, so the header-only libraries stay dependency-free.
# -----------------------------------------------------------------------------
FetchContent_Declare(
  replxx
  GIT_REPOSITORY https://github.com/AmokHuginnsson/replxx.git
  GIT_TAG release-0.0.4 # latest tagged release; also HEAD of the (unmaintained since 2021) repo
)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 11.2.0
)
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG v3.3.1
)
FetchContent_Declare(
  nlohmann_json
  # The lightweight packaged release asset (headers + a small CMakeLists.txt
  # that defines the nlohmann_json::nlohmann_json INTERFACE target), rather
  # than the full git repository with its docs/tests/tooling.
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)

set(REPLXX_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(REPLXX_BUILD_PACKAGE OFF CACHE BOOL "" FORCE)
set(FMT_TEST OFF CACHE BOOL "" FORCE)
set(FMT_DOC OFF CACHE BOOL "" FORCE)
set(CXXOPTS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CXXOPTS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)

# replxx is only ever built static here, regardless of any ambient
# BUILD_SHARED_LIBS -- save/restore it so it doesn't leak into whatever else
# FetchContent pulls in.
set(_rpc_repl_saved_bsl ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
FetchContent_MakeAvailable(replxx fmt cxxopts nlohmann_json)
set(BUILD_SHARED_LIBS ${_rpc_repl_saved_bsl})
unset(_rpc_repl_saved_bsl)

enable_testing()
add_subdirectory(extras/tests)

# -----------------------------------------------------------------------------
# rpc_repl: the interactive REPL / one-shot CLI (docs/PLAN.md component 7).
# json_value.hpp/io_worker.hpp/repl.hpp are header-only (every function is
# `inline` or an in-class member definition), so main.cpp is the only
# translation unit.
# -----------------------------------------------------------------------------
add_executable(rpc_repl extras/host/tools/rpc_repl/main.cpp)
target_link_libraries(rpc_repl PRIVATE serial_rpc replxx::replxx fmt::fmt cxxopts::cxxopts nlohmann_json::nlohmann_json)
serial_rpc_warnings(rpc_repl)

# -----------------------------------------------------------------------------
# Host serialport.hpp smoke test (PLAN.md component 4). Links two translation
# units that both include the header, so that a missing `inline` on some
# out-of-class definition shows up as a link-time multiple-definition error
# instead of silently working by luck of the build order.
# -----------------------------------------------------------------------------
add_executable(serialport_smoke
  extras/host/examples/serialport_smoke.cpp
  extras/host/examples/serialport_smoke_tu2.cpp
)
target_link_libraries(serialport_smoke PRIVATE serial_rpc)
serial_rpc_warnings(serialport_smoke)

# -----------------------------------------------------------------------------
# arduino_install (optional, not part of `all`): copies the Arduino library to
# <sketchbook>/libraries/SerialRPC and the Blink example to
# <sketchbook>/SerialRPC_Blink. The sketchbook comes from
# `arduino-cli config get directories.user` unless ARDUINO_USER_DIR is set,
# e.g. `cmake -Bbuild -DARDUINO_USER_DIR=~/Arduino`.
# -----------------------------------------------------------------------------
set(ARDUINO_USER_DIR "" CACHE PATH
    "Arduino sketchbook for arduino_install (empty: ask arduino-cli)")
add_custom_target(arduino_install
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
          -DARDUINO_USER_DIR=${ARDUINO_USER_DIR}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/extras/cmake/arduino_install.cmake
  COMMENT "Installing the SerialRPC Arduino library and Blink sketch"
  VERBATIM)
