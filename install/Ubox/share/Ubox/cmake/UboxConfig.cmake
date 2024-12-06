# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Ubox_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Ubox_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Ubox_FOUND FALSE)
  elseif(NOT Ubox_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Ubox_FOUND FALSE)
  endif()
  return()
endif()
set(_Ubox_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Ubox_FIND_QUIETLY)
  message(STATUS "Found Ubox: 0.0.0 (${Ubox_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Ubox' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT Ubox_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Ubox_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Ubox_DIR}/${_extra}")
endforeach()
