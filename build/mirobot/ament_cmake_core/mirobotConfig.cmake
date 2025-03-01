# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mirobot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mirobot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mirobot_FOUND FALSE)
  elseif(NOT mirobot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mirobot_FOUND FALSE)
  endif()
  return()
endif()
set(_mirobot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mirobot_FIND_QUIETLY)
  message(STATUS "Found mirobot: 0.3.0 (${mirobot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mirobot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mirobot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mirobot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mirobot_DIR}/${_extra}")
endforeach()
