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

# Zlib is required to decompress tracker configuration
find_package(ZLIB)

# Things we need to be able to include in our C code
include_directories(src
  ${LIBJSON_INCLUDE_DIR}
  ${LIBUSB_INCLUDE_DIR}
  ${ZLIB_INCLUDE_DIRS})

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
  deepdive)

# Add all targets to the build-tree export set
export(TARGETS deepdive deepdive_tool
  FILE "${PROJECT_BINARY_DIR}/deepdiveTargets.cmake")
 
# Export the package for use from the build-tree
export(PACKAGE deepdive)

# Create the FooBarConfig.cmake and FooBarConfigVersion files
set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(cmake/deepdiveConfig.cmake.in
  "${PROJECT_BINARY_DIR}/deepdiveConfig.cmake" @ONLY)
configure_file(cmake/deepdiveConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/deepdiveConfigVersion.cmake" @ONLY)
configure_file(cmake/deepdiveUninstall.cmake.in
  "${PROJECT_BINARY_DIR}/deepdiveUninstall.cmake" @ONLY)

# Installation, should you need to
install(TARGETS deepdive deepdive_tool
  EXPORT deepdiveTargets
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib COMPONENT shlib
  PUBLIC_HEADER DESTINATION "include/deepdive" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT deepdiveTargets DESTINATION
  share/libdeepdive COMPONENT dev)

# Install the cmake files
install(FILES
  "${PROJECT_BINARY_DIR}/deepdiveConfig.cmake"
  "${PROJECT_BINARY_DIR}/deepdiveConfigVersion.cmake"
  DESTINATION share/libdeepdive COMPONENT dev)

# Allow for the files to be uninstalled
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${PROJECT_BINARY_DIR}/deepdiveUninstall.cmake)