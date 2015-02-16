# - try to find Leap library
#
# Cache Variables: (probably not for direct use in your scripts)
#  LEAP_INCLUDE_DIR
#
# Non-cache variables you might use in your CMakeLists.txt:
#  LEAP_FOUND
#  LEAP_INCLUDE_DIRS
#  LEAP_LIBRARIES
#  LEAP_RUNTIME_LIBRARIES - aka the dll for installing
#  LEAP_RUNTIME_LIBRARY_DIRS
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# Author:
#  2014 Dave Borel <dave.borel@gmail.com>

set(LEAP_ROOT_DIR
	"${LEAP_ROOT_DIR}"
	CACHE
	PATH
	"Directory to search for Leap")

if(CMAKE_SIZEOF_VOID_P MATCHES "8")
	set(_LIBSUFFIXES /x64)
else()
	set(_LIBSUFFIXES /x86)
endif()

if(WIN32)
	set(_LIBBASENAME Leap.lib)
	set(_DLLBASENAME Leap.dll)
else()
	set(_LIBBASENAME libLeap.dylib)
	set(_DLLBASENAME libLeap.dylib)
endif()

find_path(LEAP_INCLUDE_DIR
	NAMES
	Leap.h
	HINTS
	PATHS
	"${LEAP_ROOT_DIR}"
	PATH_SUFFIXES
	include/)

set(_deps_check)

find_library(LEAP_LIBRARY
	NAMES
	${_LIBBASENAME}
	PATHS
	"${LEAP_ROOT_DIR}/lib"
	PATH_SUFFIXES
	"${_LIBSUFFIXES}")

find_file(LEAP_RUNTIME_LIBRARY
	NAMES
	${_DLLBASENAME}
	HINTS
	"${LEAP_ROOT_DIR}/lib"
	PATH_SUFFIXES
	${_LIBSUFFIXES})

set(LEAP_RUNTIME_LIBRARIES "${LEAP_RUNTIME_LIBRARY}")
get_filename_component(LEAP_RUNTIME_LIBRARY_DIRS
	"${LEAP_RUNTIME_LIBRARY}"
	PATH)
list(APPEND _deps_check LEAP_RUNTIME_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Leap
	DEFAULT_MSG
	LEAP_INCLUDE_DIR
	LEAP_RUNTIME_LIBRARY
	${_deps_check})

if(LEAP_FOUND)
	set(LEAP_INCLUDE_DIRS "${LEAP_INCLUDE_DIR}")
	mark_as_advanced(LEAP_ROOT_DIR)
endif()

mark_as_advanced(LEAP_INCLUDE_DIR
	LEAP_RUNTIME_LIBRARY)
