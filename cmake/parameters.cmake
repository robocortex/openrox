### CMake generation parameters ###
### Here are defined the cmake behaviors ###

MESSAGE(STATUS "\n_____ rox_open/cmake/parameters.cmake ____________________________________________\n" )

# By default, Use "Release" mode
#if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug, Release"   FORCE)
#endif()

# Is this build for external tracking release ?
option(OPENROX_BUILD_RELEASE                    "build external (public) release for OPENROX"              OFF)

# Is this build use an internal memory pool ?
option(OPENROX_USES_MEMORY_POOL                  "use an internal memory pool for OPENROX allocations"      OFF)

# Is this build verbose on screen ?
option(OPENROX_VERBOSE_DISPLAY                   "display logs on standard output"                          OFF)

# Is this build verbose on file ?
option(OPENROX_VERBOSE_LOGFILE                   "write logs in file"                                       OFF)

# Is this build track rox_memory allocation ?
option(OPENROX_VERBOSE_MEMORY                    "display rox_memory calls"                                 OFF)

option(OPENROX_LOGS 							          "enable rox_log" 										             ON)

#cmake_dependent_option(OPENROX_CREATE_MANUAL_PROG "Use Doxygen to create the HTML based API documentation"  OFF OPENROX_BUILD_RELEASE ON)
#cmake_dependent_option(OPENROX_CREATE_MANUAL_USER "Use pdflatex to build the user manual"                   OFF OPENROX_BUILD_RELEASE ON)

option(OPENROX_CREATE_MANUAL_PROG             "Use Doxygen to create the HTML based API documentation"   ON)
option(OPENROX_CREATE_MANUAL_USER             "Use pdflatex to build the user manual"                    ON)

if (OPENROX_HAS_STD_EXAMPLES_SUPPORT)
   OPTION ( OPENROX_BUILD_EXAMPLES "Build examples" OFF )
else()
   MESSAGE(STATUS "CURRENT PLATFORM DOES NOT HAVE STD EXAMPLES SUPPORT")
endif ()

if (OPENROX_VERBOSE_LOGFILE)
   add_definitions(-DROX_LOGFILE)
endif()

if (OPENROX_VERBOSE_MEMORY)
   add_definitions(-DROX_LOGMEMORY)
endif()

if (OPENROX_LOGS)
   add_definitions(-DROX_LOGS)
endif()

if(OPENROX_VERBOSE_DISPLAY)
   message("OPENROX_VERBOSE_DISPLAY: " ${OPENROX_VERBOSE_DISPLAY})
endif()

if (OPENROX_VERBOSE_DISPLAY)
   message("Add definition -DROX_DISPLAY")
   add_definitions(-DROX_DISPLAY)
endif()

if(NOT PIKEOS)
   add_definitions(-DROX_USE_CTYPE)
endif()

# Enable openrox log display for examples and tests 
if(OPENROX_BUILD_TESTS OR OPENROX_BUILD_EXAMPLES)
   add_definitions(-DROX_LOGS)
endif()

# Define folder with data for examples and tests
add_definitions(-DROX_DATA_HOME="$ENV{OPENROX_DATA_HOME}")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# By default, compile as shared
if (OPENROX_HAS_SHAREDSTATIC_SUPPORT MATCHES "static")
   set(OPENROX_CREATE_SHARED_LIBRARIES OFF     )
   set(LIB_TYPE                    "static")
elseif (OPENROX_HAS_SHAREDSTATIC_SUPPORT MATCHES "shared")
   set(OPENROX_CREATE_SHARED_LIBRARIES ON      )
   set(LIB_TYPE                    "shared")
else()
   set(LIB_TYPE "shared")
   option(OPENROX_CREATE_SHARED_LIBRARIES "build libraries as shared libraries" ON)
endif()

# message("OPENROX_HAS_OPENMP_SUPPORT = " ${OPENROX_HAS_OPENMP_SUPPORT})

# Does this build make use of OpenMP ?
if (OPENROX_HAS_OPENMP_SUPPORT)
   option(OPENROX_USES_OPENMP                   "${PROJECT_NAME} is using OpenMP"                      ON)
endif()

# Does this build make use of GPROF ?
if (OPENROX_HAS_GPROF_SUPPORT)
   option(OPENROX_USES_GPROF                    "using Gprof"                                         OFF)
endif()

if(ANDROID)
   set(OPENROX_DATA_HOME /storage/emulated/0/Documents/Openrox/Tests/Datasets CACHE STRING "directory for data")
else()
   set(OPENROX_DATA_HOME $ENV{OPENROX_DATA_HOME} CACHE STRING "directory for data")
endif()

if (NOT OPENROX_BUILD_RELEASE)
   # Do we use platform specific optimizations
   option(OPENROX_CREATE_PLATFORM_OPTIMIZED                  "build optimized release" ON)

   # Do we use platform specific linear algebra
   option(OPENROX_CREATE_PLATFORM_OPTIMIZED_LINEAR_ALGEBRA   "use optimized linear algebra" ON)

   if(OPENROX_CREATE_PLATFORM_OPTIMIZED)
      option(OPENROX_CREATE_PLATFORM_OPTIMIZED_AVX "build optimized AVX release" OFF)
   else()
      set(OPENROX_CREATE_PLATFORM_OPTIMIZED_AVX OFF CACHE BOOL "build optimized AVX release" FORCE)
   endif()

else()
   # Do we use platform specific optimizations
   set(OPENROX_CREATE_PLATFORM_OPTIMIZED                ON)
   # Do we use platform specific linear algebra
   set(OPENROX_CREATE_PLATFORM_OPTIMIZED_LINEAR_ALGEBRA ON)

   if(OPENROX_CREATE_PLATFORM_OPTIMIZED)
      option(OPENROX_CREATE_PLATFORM_OPTIMIZED_AVX "build optimized AVX release" OFF)
   else()
      set(OPENROX_CREATE_PLATFORM_OPTIMIZED_AVX OFF CACHE BOOL "build optimized AVX release" FORCE)
   endif()

   option(OPENROX_CREATE_MANUAL_PROG             "Use Doxygen to create the HTML based API documentation"   ON)

   option(OPENROX_CREATE_MANUAL_USER             "Use pdflatex to build the user manual"                    ON)

endif()

if ( OPENROX_CREATE_SHARED_LIBRARIES )
   add_definitions("-DROX_EXPORT_SHARED")
else()
   add_definitions("-DROX_EXPORT_STATIC")
endif()
