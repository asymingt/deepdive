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
  ${EIGEN3_INCLUDE_DIRS})

# Bootstrap catkin_simple
catkin_simple()

# Bridge converts USB tracker data to ROS messages
cs_add_executable(deepdive_bridge src/deepdive_bridge.cc)
target_link_libraries(deepdive_bridge ${DEEPDIVE_LIBRARIES})

# Filter find the world pose of a soecific tracker
cs_add_executable(deepdive_filter src/deepdive_filter.cc)
add_dependencies(deepdive_filter ukf)
target_compile_definitions(deepdive_filter
  PRIVATE -DUKF_DOUBLE_PRECISION)

# Solver finds the world pose of every lighthouse
#cs_add_executable(deepdive_solver src/deepdive_solver.cc)
#target_link_libraries(deepdive_solver ${CERES_LIBRARIES})

# Fusion merges multiple tracker solutions to track a rigid body
cs_add_executable(deepdive_fusion src/deepdive_fusion.cc)

# Install products
cs_install()

# Export targets
cs_export()