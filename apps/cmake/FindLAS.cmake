# Locate the LAS library
# This module defines
# LAS_FOUND, if false do not try to link against LAS
# LAS_LIBRARY, the name of the library to link against
# LAS_INCLUDE_DIR, the liblas include directory, eg headers are under liblas/
#
# You can also specify the environment variable LAS_DIR to direct
# the module where to search for libLAS if it's installed in a non-standard location
# alternatively you can pass -DLAS_DIR=/path/to/libLAS when configuring and this path will
# be searched

FIND_PATH(LAS_INCLUDE_DIR liblas.hpp
	HINTS
	$ENV{LAS_DIR}
	${LAS_DIR}
	PATH_SUFFIXES include/liblas include liblas
	PATHS
	~/Library/Frameworks
	/Library/Frameworks
	/usr/local/include/liblas
	/usr/include/liblas
	/sw # Fink
	/opt/local # DarwinPorts
	/opt/csw # Blastwave
	/opt
)
# LAS actually expects its own includes to be under liblas/ as well so we need to go up one
get_filename_component(LAS_INCLUDE_DIR ${LAS_INCLUDE_DIR} PATH)

# Lookup the 64 bit libs on x64
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	FIND_LIBRARY(LAS_LIBRARY_TEMP las
		HINTS
		${LAS_DIR}
		$ENV{LAS_DIR}
		PATH_SUFFIXES lib64 lib
		lib/x64
		PATHS
		/sw
		/opt/local
		/opt/csw
		/opt
	)
# On 32bit build find the 32bit libs
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
	FIND_LIBRARY(LAS_LIBRARY_TEMP las
		HINTS
		${LAS_DIR}
		$ENV{LAS_DIR}
		PATH_SUFFIXES lib
		lib/x86
		PATHS
		/sw
		/opt/local
		/opt/csw
		/opt
	)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

SET(LAS_FOUND "NO")
IF(LAS_LIBRARY_TEMP)
	# Set the final string here so the GUI reflects the final state.
	SET(LAS_LIBRARY ${LAS_LIBRARY_TEMP} CACHE STRING "Where the LAS Library can be found")
	# Set the temp variable to INTERNAL so it is not seen in the CMake GUI
	SET(LAS_LIBRARY_TEMP "${LAS_LIBRARY_TEMP}" CACHE INTERNAL "")

	SET(LAS_FOUND "YES")
ENDIF(LAS_LIBRARY_TEMP)

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(LAS REQUIRED_VARS LAS_LIBRARY LAS_INCLUDE_DIR)

