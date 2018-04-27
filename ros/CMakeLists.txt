# Don't try and compile with older cmake versions
cmake_minimum_required(VERSION 3.5)

# Name of this package
project(deepdive_ros)

# We will be using c++11 throughout our code
add_definitions(-std=c++14)

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

# Find Eigen
find_package(Eigen3 REQUIRED)

# Find OpenCV3
find_package(OpenCV 3 REQUIRED)

# Non-linear least squares optimization (apt install libceres-dev)
find_package(Ceres REQUIRED)

# Low-level driver for Vive
find_package(Deepdive REQUIRED)

# Find catkin simple
find_package(catkin_simple REQUIRED)

# Setup the include directories
include_directories(src
  ${DEEPDIVE_INCLUDE_DIRS}
  ${UKF_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# Bootstrap catkin_simple
catkin_simple()

# Bridge converts USB tracker data to ROS messages
cs_add_executable(deepdive_bridge src/deepdive_bridge.cc)
target_link_libraries(deepdive_bridge ${DEEPDIVE_LIBRARIES})

# Core library
cs_add_library(deepdive_core src/deepdive.cc)

# Solver finds the world pose of every lighthouse
cs_add_executable(deepdive_calibration src/deepdive_calibration.cc)
target_link_libraries(deepdive_calibration deepdive_core ${OpenCV_LIBS})

# Solver finds the world pose of every lighthouse
cs_add_executable(deepdive_registration src/deepdive_registration.cc)
target_link_libraries(deepdive_registration deepdive_core ${CERES_LIBRARIES})

# Filter find the world pose of a soecific tracker
# cs_add_executable(deepdive_tracker src/deepdive_tracker.cc)
# target_compile_definitions(deepdive_tracker PRIVATE -DUKF_DOUBLE_PRECISION)
# target_link_libraries(deepdive_tracker deepdive_core ${CERES_LIBRARIES})
# add_dependencies(deepdive_tracker ukf)

# Install products
cs_install()

# Export targets
cs_export()