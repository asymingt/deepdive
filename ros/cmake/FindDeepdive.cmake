# - Try to find libdeepdive
# Will define
# DEEPDIVE_FOUND
# DEEPDIVE_INCLUDE_DIRS
# DEEPDIVE_LIBRARIES

find_path(DEEPDIVE_INCLUDE_DIRS deepdive/deepdive.h
  HINTS ${DEEPDIVE_INCLUDEDIR})

find_library(DEEPDIVE_LIBRARIES
  NAMES deepdive
  HINTS ${DEEPDIVE_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(deepdive DEFAULT_MSG DEEPDIVE_LIBRARIES DEEPDIVE_INCLUDE_DIRS)
mark_as_advanced(DEEPDIVE_LIBRARIES DEEPDIVE_INCLUDE_DIRS)
