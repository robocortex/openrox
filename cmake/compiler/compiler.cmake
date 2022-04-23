MESSAGE(STATUS "\n_____ rox_open/cmake/compiler.cmake ____________________________________________\n" )

# Configure projects according to selected compiler
#
# This module defines:
# CMAKE_C_FLAGS_DEBUG C flags for debug build
# CMAKE_C_FLAGS_Release C flags for release build
# CMAKE_CXX_FLAGS_DEBUG CXX flags for debug build
# CMAKE_CXX_FLAGS_Release CXX flags for release build
# ROX_API A C Macro used for release
#
# Note: To allow Mono and Unity project, it is required to change CMAKE_C_COMPILER_ID
#       for instance: set(CMAKE_C_COMPILER_ID Mono)
#
# Warning: Be careful to revert this change after, or to find dependencies before it.
#          as this is a CMake variable which go through "add_subdirectory" routine.
#          So it can occured some strange behavior in the concerned dependencies.
#          (for instance, if the dependencies is a C++ library, it will be managed as a Mono one.)

if ( NOT CMAKE_BUILD_TYPE )
   # Define configuration types
   if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
      set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo CACHE TYPE INTERNAL FORCE)
   else()
      set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo CACHE STRING INTERNAL FORCE)
   endif()

   #By default, use "Release" mode
   # note: This variable is ignored by visual as it uses multi configurations.
   set ( CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug, Release" FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

### Per compiler configuration ###
### This is a hub to add options to the right compiler ###

if ("\"${CMAKE_C_COMPILER_ID}\"" MATCHES "GNU")
   #GCC Compiler
   include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_gnucc.cmake)
elseif ("\"${CMAKE_C_COMPILER_ID}\"" MATCHES  "Clang")
   #CLANG LLVM compiler
   include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_clang.cmake)
elseif ("\"${CMAKE_C_COMPILER_ID}\"" MATCHES MSVC)

   if     (MSVC_VERSION EQUAL  1600)
      set(MSVC10 true)
      include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_msvc10_2010.cmake)
   elseif (MSVC_VERSION EQUAL  1700)
      set(MSVC11 true)
      include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_msvc11_2012.cmake)
   elseif (MSVC_VERSION EQUAL  1800)
      set(MSVC12 true)
      include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_msvc12_2013.cmake)
   elseif (MSVC_VERSION EQUAL  1900)
      set(MSVC14 true)
      include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_msvc14_2015.cmake)
   elseif (MSVC_VERSION LESS_EQUAL 1929 AND MSVC_VERSION GREATER_EQUAL 1920)
      set(MSVC16 true)
      include(${OPENROX_SOURCE_DIR}/cmake/compiler/compiler_msvc16_2019.cmake)
   else ()
   #Compiler need to be specified and known, otherwise it is a fatal error
   message(FATAL_ERROR "Compiler not supported")
   endif ()
else ()
   #Compiler need to be specified and known, otherwise it is a fatal error
   message(FATAL_ERROR "Compiler not supported")
endif ()
