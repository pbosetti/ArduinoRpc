# SPDX-License-Identifier: Apache-2.0
#
# Installs the SerialRPC Arduino library (and its Blink example as an editable
# sketch) into the local Arduino sketchbook. Run through the `arduino_install`
# target (`cmake --build build --target arduino_install`), which invokes:
#
#   cmake -DSOURCE_DIR=<repo root> [-DARDUINO_USER_DIR=<sketchbook>] -P <this>
#
# The sketchbook is ARDUINO_USER_DIR if given, else what arduino-cli reports
# as `directories.user`, else the IDE default for the platform.
#
# - The library goes to <sketchbook>/libraries/SerialRPC, replacing any
#   previous SerialRPC install there (the IDE lists its examples under
#   File > Examples > SerialRPC). extras/ is not installed: it is host-only.
# - Blink is copied to <sketchbook>/SerialRPC_Blink/SerialRPC_Blink.ino only
#   if that sketch does not exist yet, so local edits are never overwritten.

cmake_minimum_required(VERSION 3.24)

if(NOT SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR not set")
endif()

# --- Locate the sketchbook ---------------------------------------------------
if(NOT ARDUINO_USER_DIR)
  find_program(ARDUINO_CLI arduino-cli)
  if(ARDUINO_CLI)
    execute_process(
      COMMAND ${ARDUINO_CLI} config get directories.user
      OUTPUT_VARIABLE ARDUINO_USER_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _rc ERROR_QUIET)
    if(NOT _rc EQUAL 0)
      set(ARDUINO_USER_DIR "")
    endif()
  endif()
endif()
if(NOT ARDUINO_USER_DIR)
  if(WIN32 OR APPLE)
    file(TO_CMAKE_PATH "$ENV{HOME}/Documents/Arduino" ARDUINO_USER_DIR)
    if(WIN32)
      file(TO_CMAKE_PATH "$ENV{USERPROFILE}/Documents/Arduino"
           ARDUINO_USER_DIR)
    endif()
  else()
    set(ARDUINO_USER_DIR "$ENV{HOME}/Arduino")
  endif()
endif()
file(TO_CMAKE_PATH "${ARDUINO_USER_DIR}" ARDUINO_USER_DIR)
message(STATUS "Arduino sketchbook: ${ARDUINO_USER_DIR}")

# --- Library -----------------------------------------------------------------
set(lib_dir "${ARDUINO_USER_DIR}/libraries/SerialRPC")
if(EXISTS "${lib_dir}")
  # Only ever replace a previous SerialRPC install, never an unrelated folder.
  set(_props "${lib_dir}/library.properties")
  if(EXISTS "${_props}")
    file(STRINGS "${_props}" _name REGEX "^name=")
  endif()
  if(NOT _name STREQUAL "name=SerialRPC")
    message(FATAL_ERROR "${lib_dir} exists but is not a SerialRPC install; "
                        "refusing to overwrite it")
  endif()
  file(REMOVE_RECURSE "${lib_dir}")
endif()
file(MAKE_DIRECTORY "${lib_dir}")
file(COPY "${SOURCE_DIR}/library.properties" "${SOURCE_DIR}/keywords.txt"
          "${SOURCE_DIR}/LICENSE" "${SOURCE_DIR}/README.md"
          "${SOURCE_DIR}/src" "${SOURCE_DIR}/examples"
     DESTINATION "${lib_dir}")
file(STRINGS "${SOURCE_DIR}/library.properties" _version REGEX "^version=")
string(REPLACE "version=" "" _version "${_version}")
message(STATUS "Installed SerialRPC ${_version} -> ${lib_dir}")

# --- Blink sketch --------------------------------------------------------------
set(sketch_dir "${ARDUINO_USER_DIR}/SerialRPC_Blink")
if(EXISTS "${sketch_dir}")
  message(STATUS "Kept existing sketch ${sketch_dir} (not overwritten)")
else()
  file(MAKE_DIRECTORY "${sketch_dir}")
  configure_file("${SOURCE_DIR}/examples/Blink/Blink.ino"
                 "${sketch_dir}/SerialRPC_Blink.ino" COPYONLY)
  message(STATUS "Installed sketch -> ${sketch_dir}/SerialRPC_Blink.ino")
endif()
