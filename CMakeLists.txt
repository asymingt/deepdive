# Miniumum required cmake file
cmake_minimum_required(VERSION 3.5)

# Deepdive is a fork of libsurvive for precisino, tracker-only support
# This will generate PROJECT_VERSION_* files automatically for us
project(deepdive VERSION 0.1.0 LANGUAGES C)

# Libusb is needed to interact with the devices
find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY NAMES usb-1.0 PATH_SUFFIXES "lib" "lib32" "lib64")

# JSON is needed to parse the trackr configuration
find_path(LIBJSON_INCLUDE_DIR NAMES json.h PATH_SUFFIXES "include" "json")
find_library(LIBJSON_LIBRARY NAMES json PATH_SUFFIXES "lib" "lib32" "lib64")

# argtable2 is for argument parsing of the commandline
find_path(ARGTABLE2_INCLUDE_DIRS argtable2.h PATH_SUFFIXES "include")
find_library(ARGTABLE2_LIBRARY argtable2 PATH_SUFFIXES "lib") 

# Zlib is required to decompress tracker configuration
find_package(ZLIB)

# Things we need to be able to include in our C code
include_directories(src
  ${LIBJSON_INCLUDE_DIR}
  ${LIBUSB_INCLUDE_DIR}
  ${ZLIB_INCLUDE_DIRS}
  ${ARGTABLE2_INCLUDE_DIRS})

# Core library does the heavy-lifting
add_library(deepdive SHARED
  src/deepdive.h
  src/deepdive.c
  src/deepdive_dev_tracker.c
  src/deepdive_dev_watchman.c
  src/deepdive_data_light.c
  src/deepdive_data_imu.c
  src/deepdive_data_button.c
  src/deepdive_usb.c)
target_link_libraries(deepdive
  ${LIBJSON_LIBRARY}
  ${LIBUSB_LIBRARY}
  ${ZLIB_LIBRARIES})
set_target_properties(deepdive PROPERTIES
  PUBLIC_HEADER src/deepdive.h)

# Simple tool to test the library
add_executable(deepdive_tool
  src/deepdive_tool.c)
target_link_libraries(deepdive_tool
  deepdive
  ${ARGTABLE2_LIBRARY})

# Create an uninstall script for covenience
configure_file(cmake/deepdiveUninstall.cmake.in
  "${PROJECT_BINARY_DIR}/deepdiveUninstall.cmake" @ONLY)

# Installation, should you need to
install(TARGETS deepdive deepdive_tool
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION "include/deepdive" COMPONENT dev)

# Allow for the files to be uninstalled
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${PROJECT_BINARY_DIR}/deepdiveUninstall.cmake)