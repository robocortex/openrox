# Locate openrox and add it to your build process
#
# This module defines:
# OPENROX_LIBRARIES use this with add_dependencies on your project
# OPENROX_INCLUDE_DIRS use this with include_directories on your project
# OPENROX_CSHARP_PLUGIN_DIR directory containing all c# interfaces to the plugins
# OPENROX_C_PLUGIN_DIR directory containing all c interfaces to the plugins
# OPENROX_BINARIES_DIR where to find all generated binaries, use this to create a post build event that copies binaries to output folder
# OPENROX_BINARIES_DIR_DEBUG where to find generated binaries in debug version, use this to create a post build event that copies binaries to output folder
# OPENROX_BINARIES_DIR_RELEASE where to find generated binaries in release version, use this to create a post build event that copies binaries to output folder
#
# $OPENROX_HOME is an environment variable that would correspond to the openrox root folder


#if already present in global project, do not add it
if(NOT TARGET openrox)
   MESSAGE(STATUS "\n________________________________________ OpenRox ______________________________________\n" )
   add_subdirectory("$ENV{OPENROX_HOME}" ${CMAKE_BINARY_DIR}/externals/openrox)
   set_target_properties (openrox PROPERTIES FOLDER Dependencies)
   set_property(GLOBAL PROPERTY USE_FOLDERS ON)
   set(OPENROX_INCLUDE_DIRS ${OPENROX_SOURCE_DIR}/sources;${CMAKE_BINARY_DIR}/externals/openrox CACHE INTERNAL "Include directories for OPENROX")
   set(OPENROX_BINARIES_DIR ${CMAKE_BINARY_DIR}/externals/openrox CACHE INTERNAL "Binary directoris for OPENROX")
endif()


set(OPENROX_LIBRARIES openrox)
set(OPENROX_BINARIES_DIR_DEBUG ${OPENROX_BINARIES_DIR}/Debug)
set(OPENROX_BINARIES_DIR_RELEASE ${OPENROX_BINARIES_DIR}/Release)
set(OPENROX_EXAMPLES_DIRS ${OPENROX_SOURCE_DIR}/examples)
