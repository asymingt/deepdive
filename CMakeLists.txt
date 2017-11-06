cmake_minimum_required(VERSION 3.2)

# Deepdive is a fork of libsurvive for precisino, tracker-only support
project(libdeepdive VERSION 0.0.1 LANGUAGES C)

# Libusb is needed to interact with the devices
find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h
  PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY NAMES usb-1.0
  PATH_SUFFIXES "lib" "lib32" "lib64")

# JSON is needed to parse the trackr configuration
find_path(LIBJSON_INCLUDE_DIR NAMES json.h
  PATH_SUFFIXES "include" "json")
find_library(LIBJSON_LIBRARY NAMES json
  PATH_SUFFIXES "lib" "lib32" "lib64")

# Zlib is required to decompress tracker configuration
find_package(ZLIB)

# Things we need to be able to include in our C code
include_directories(
  include/libdeepdive
  ${LIBJSON_INCLUDE_DIR}
  ${LIBUSB_INCLUDE_DIR}
  ${ZLIB_INCLUDE_DIRS}
)

# Core library does the heavy-lifting
add_library(deepdive SHARED
  src/deepdive.c
  src/deepdive_dev_tracker.c
  src/deepdive_dev_watchman.c
  src/deepdive_data_light.c
  src/deepdive_data_imu.c
  src/deepdive_data_button.c
  src/deepdive_usb.c
)
target_link_libraries(deepdive
  ${LIBJSON_LIBRARY}
  ${LIBUSB_LIBRARY}
  ${ZLIB_LIBRARIES}
)

# Simple tool to test the library
add_executable(test_tool src/test_tool.c)
target_link_libraries(test_tool deepdive)

# Installation, should you need to
install(TARGETS deepdive DESTINATION lib)
install(TARGETS test_tool DESTINATION bin)
install(DIRECTORY include/libdeepdive DESTINATION include)