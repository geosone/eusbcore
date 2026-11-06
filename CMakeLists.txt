# SPDX-License-Identifier: GPL-2.0-or-later
#
# eusbcore: the native USB stack (C90) - xHCI, EHCI, OHCI and UHCI host
# controllers, enumeration and hubs - for 32/64-bit kernels such as
# GonWare's GDOS. Class drivers: EUSBASPI (mass storage), openinput (HID).
#
# Needs OpenASPI's host services (target OpenASPI::host): a parent project
# provides it, or OPENASPI_SOURCE_DIR points to an OpenASPI checkout
# (default: ../OpenAspi or ../OpenASPI next to this one).

cmake_minimum_required(VERSION 3.20)
project(eusbcore LANGUAGES C)

if(NOT TARGET OpenASPI::host)
  set(OPENASPI_SOURCE_DIR "" CACHE PATH "OpenASPI source tree")
  if(NOT OPENASPI_SOURCE_DIR)
    foreach(_d OpenAspi OpenASPI)
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../${_d}/CMakeLists.txt")
        set(OPENASPI_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../${_d}")
      endif()
    endforeach()
  endif()
  if(NOT OPENASPI_SOURCE_DIR)
    message(FATAL_ERROR "eusbcore: set OPENASPI_SOURCE_DIR to an OpenASPI checkout")
  endif()
  add_subdirectory("${OPENASPI_SOURCE_DIR}" openaspi EXCLUDE_FROM_ALL)
endif()

add_library(eusbcore STATIC src/usb.c src/xhci.c src/ehci.c src/ohci.c src/uhci.c)
target_include_directories(eusbcore PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(eusbcore PUBLIC OpenASPI::host)
set_target_properties(eusbcore PROPERTIES C_STANDARD 90 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
add_library(EUSBCORE::core ALIAS eusbcore)

if(PROJECT_IS_TOP_LEVEL)
  target_compile_options(eusbcore PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
