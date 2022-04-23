MESSAGE(STATUS "\n_____ rox_open/cmake/compiler_gnucc.cmake ____________________________________________\n" )
# GNU GCC Configuration

SET(COMPILER "gcc")

# CLEAN FLAGS
SET(${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS       "")
SET(${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "")

SET(${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "")
SET(${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "")

# Debug flags
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   "-DDEBUG")
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-DDEBUG")

# All warnings activated
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS       "-std=c99 -pedantic -fPIC -Wall -fvisibility=hidden -s")
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "-std=c99 -pedantic -fPIC -Wall -g")

list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-std=c++14 -fPIC -Wall -Wno-unknown-pragmas -fvisibility=hidden -s")
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-std=c++14 -fPIC -Wall -Wno-unknown-pragmas -g")

# Optimize for current platform
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS   "-O3")
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS "-O3")

if (OPENROX_HAS_AVX_SUPPORT AND ${PROJECT_NAME}_CREATE_PLATFORM_OPTIMIZED_AVX)
   set (OPENROX_USE_AVX true)
   add_definitions("-DROX_USE_AVX")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS       "-msse4.2 -mavx")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "-msse4.2 -mavx")

   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-msse4.2 -mavx")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-msse4.2 -mavx")

elseif (OPENROX_HAS_SSE_SUPPORT AND ${PROJECT_NAME}_CREATE_PLATFORM_OPTIMIZED)
   set (OPENROX_USE_SSE true)
   add_definitions("-DROX_USE_SSE")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS       "-msse4.2")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "-msse4.2")

   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-msse4.2")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-msse4.2")

elseif (OPENROX_HAS_NEON_SUPPORT AND ${PROJECT_NAME}_CREATE_PLATFORM_OPTIMIZED)

   set (OPENROX_USE_NEON true)
   add_definitions("-DROX_USE_NEON")

  if (OPENROX_IS_IOS)
     list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS         "-arch armv7")
     list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   "-arch armv7")
     list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-arch armv7")
     list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-arch armv7")
   endif()

   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS       "-marm")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS       "-mfpu=neon")

   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "-marm")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "-mfpu=neon")

   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-marm")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-mfpu=neon")

   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-marm")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-mfpu=neon")

   if (OPENROX_IS_LINUX)
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS         "-mcpu=cortex-a9")
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   "-mcpu=cortex-a9")
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-mcpu=cortex-a9")
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-mcpu=cortex-a9")
   else()
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS         "-mfloat-abi=softfp")
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   "-mfloat-abi=softfp")

      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-mfloat-abi=softfp")
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-mfloat-abi=softfp")
   endif()

endif()

# OpenMP options and flags
if (OPENROX_HAS_OPENMP_SUPPORT AND ${PROJECT_NAME}_USES_OPENMP)
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS         "-fopenmp")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   "-fopenmp")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-fopenmp")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-fopenmp")
   add_definitions("-DROX_USES_OPENMP")
   MESSAGE("Adding openmp in ${PROJECT_NAME}" )
else()
   MESSAGE("NOT adding openmp in ${PROJECT_NAME}" )
endif()

# Gprof options and flags
if (OPENROX_HAS_GPROF_SUPPORT AND ${PROJECT_NAME}_USES_GPROF)
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS         "-g -pg")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   "-g -pg")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       "-g -pg")
   list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG "-g -pg")
   add_definitions("-DROX_USES_GPROF")
   MESSAGE("Add gprof" )
endif()

if(NOT TOOLCHAIN_SETS_CMAKE_FLAGS)
   # Create argument lists for debug
   foreach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG})
      set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${param}")
   endforeach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG})

   foreach(param ${${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG})
      set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${param}")
   endforeach(param ${${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG})

   # Create argument lists for release
   foreach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS})
      set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${param}")
   endforeach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS})

   foreach(param ${${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS})
      set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${param}")
   endforeach(param ${${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS})
endif()

if (${PROJECT_NAME}_CREATE_SHARED_LIBRARIES)
   set (ROX_API "__attribute__((visibility(\"default\")))")
else ()
   set (ROX_API "")
endif ()
