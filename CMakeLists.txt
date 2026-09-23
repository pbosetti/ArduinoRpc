cmake_minimum_required(VERSION 3.24)
project(arduino_rpc CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Shared codec + (later) framing/server/host-RPC headers, available to both
# the host build and, via arduino-cli --library, the Arduino build.
add_library(serial_rpc INTERFACE)
target_include_directories(serial_rpc INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/arduino/SerialRPC/src
  ${CMAKE_CURRENT_SOURCE_DIR}/host/include
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

# fmt and cxxopts are added via FetchContent in a later step, once
# host/serial_rpc.hpp and host/examples/rpc_cli.cpp exist.

enable_testing()
add_subdirectory(tests)

# -----------------------------------------------------------------------------
# Host serialport.hpp smoke test (PLAN.md component 4). Links two translation
# units that both include the header, so that a missing `inline` on some
# out-of-class definition shows up as a link-time multiple-definition error
# instead of silently working by luck of the build order.
# -----------------------------------------------------------------------------
add_executable(serialport_smoke
  host/examples/serialport_smoke.cpp
  host/examples/serialport_smoke_tu2.cpp
)
target_link_libraries(serialport_smoke PRIVATE serial_rpc)
target_compile_options(serialport_smoke PRIVATE -Wall -Wextra -Wpedantic)
