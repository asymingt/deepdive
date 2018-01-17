# Don't try and compile with older cmake versions
cmake_minimum_required(VERSION 3.5)

# Name of this package
project(deepdive_ros)

# We will be using c++11 throughout our code
add_definitions(-std=c++11)

# The ability to build external projects
include(ExternalProject)

# Use our cmake scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

## EXTERNAL SOURCES

# libukf - a C++ implementation of the unscented kalman filter
ExternalProject_Add(ukf
  GIT_REPOSITORY https://github.com/sfwa/ukf.git
  GIT_TAG master
  INSTALL_COMMAND "")
ExternalProject_Get_Property(ukf source_dir)
set(UKF_INCLUDE_DIRS ${source_dir}/include)

# Non-linear least squares optimization (apt install libceres-dev)
find_package(Ceres REQUIRED)

# Low-level driver for Vive
find_package(Deepdive REQUIRED)

# Find catkin simple
find_package(catkin_simple REQUIRED)

# Setup the include directories
include_directories(
  ${DEEPDIVE_INCLUDE_DIRS}
  ${UKF_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

## THIS PROJECT

message("${DEEPDIVE_INCLUDE_DIRS}")

catkin_simple()

cs_add_executable(deepdive_bridge src/deepdive_bridge.cc)
target_link_libraries(deepdive_bridge ${DEEPDIVE_LIBRARIES})

cs_add_executable(deepdive_solver src/deepdive_solver.cc)
target_link_libraries(deepdive_solver ${CERES_LIBRARIES})

cs_add_executable(deepdive_filter src/deepdive_filter.cc)
add_dependencies(deepdive_filter ukf)

cs_install()

cs_export()