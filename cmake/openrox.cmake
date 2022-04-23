### First of all, set cmake paths ###

MESSAGE(STATUS "\n_____ rox_open/cmake/openrox.cmake ____________________________________________\n" )

# Define version. To be changed on each release
set (OPENROX_MAJOR_VERSION 1)
set (OPENROX_MINOR_VERSION 0)
set (OPENROX_PATCH_VERSION 0)
set (OPENROX_VERSION ${OPENROX_MAJOR_VERSION}_${OPENROX_MINOR_VERSION}_${OPENROX_PATCH_VERSION})

add_definitions(-DOPENROX_MAJOR_VERSION="${OPENROX_MAJOR_VERSION}")
add_definitions(-DOPENROX_MINOR_VERSION="${OPENROX_MINOR_VERSION}")
add_definitions(-DOPENROX_PATCH_VERSION="${OPENROX_PATCH_VERSION}")

set (OPENROX_CMAKE_DIR ${OPENROX_SOURCE_DIR}/cmake)

### Cmake modules path (To find external libs and include cmake files) ###
list(APPEND CMAKE_MODULE_PATH ${OPENROX_CMAKE_DIR})
list(APPEND CMAKE_MODULE_PATH ${OPENROX_CMAKE_DIR}/CMakeModules)

# Define platform dependent cmake behavior and variables
include(${OPENROX_CMAKE_DIR}/platform/platform.cmake)

# Define macros
include(${OPENROX_CMAKE_DIR}/macros.cmake)

# Define project parameters
include(${OPENROX_CMAKE_DIR}/parameters.cmake)

# Define compiler parameters
include(${OPENROX_CMAKE_DIR}/compiler/compiler.cmake)

# External dependencies
include(${OPENROX_CMAKE_DIR}/dependencies.cmake)

# Core library building
include(${OPENROX_CMAKE_DIR}/openrox_lib.cmake)

# Tests for library
include(${OPENROX_CMAKE_DIR}/tests.cmake)

# Examples for library
include(${OPENROX_CMAKE_DIR}/examples.cmake)

# Output parameters for release
include(${OPENROX_CMAKE_DIR}/install/install.cmake)

