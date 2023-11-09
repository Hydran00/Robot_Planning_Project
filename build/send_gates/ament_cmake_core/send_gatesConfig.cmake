# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_send_gates_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED send_gates_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(send_gates_FOUND FALSE)
  elseif(NOT send_gates_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(send_gates_FOUND FALSE)
  endif()
  return()
endif()
set(_send_gates_CONFIG_INCLUDED TRUE)

# output package information
if(NOT send_gates_FIND_QUIETLY)
  message(STATUS "Found send_gates: 0.0.0 (${send_gates_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'send_gates' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${send_gates_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(send_gates_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${send_gates_DIR}/${_extra}")
endforeach()
