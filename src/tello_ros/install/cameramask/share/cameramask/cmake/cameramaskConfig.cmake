# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cameramask_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cameramask_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cameramask_FOUND FALSE)
  elseif(NOT cameramask_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cameramask_FOUND FALSE)
  endif()
  return()
endif()
set(_cameramask_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cameramask_FIND_QUIETLY)
  message(STATUS "Found cameramask: 0.0.0 (${cameramask_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cameramask' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cameramask_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cameramask_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cameramask_DIR}/${_extra}")
endforeach()
