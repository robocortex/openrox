MESSAGE(STATUS "\n_____ rox_open/cmake/compiler_msvc11_2012.cmake ____________________________________________\n" )

# MSVC11_2012 Configuration
set(COMPILER "msvc11_2012")

SET_SOURCE_FILES_PROPERTIES( *.c PROPERTIES LANGUAGE CXX )

#CLEAN FLAGS
SET(${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS "")
SET(${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "")

if (OPENROX_HAS_AVX_SUPPORT AND ${PROJECT_NAME}_CREATE_PLATFORM_OPTIMIZED_AVX)
   set (OPENROX_USE_AVX true)
   add_definitions("/DROX_USE_AVX")
elseif(OPENROX_HAS_SSE_SUPPORT AND ${PROJECT_NAME}_CREATE_PLATFORM_OPTIMIZED)
   set (OPENROX_USE_SSE true)
   add_definitions("/DROX_USE_SSE")
elseif (OPENROX_HAS_NEON_SUPPORT AND ${PROJECT_NAME}_CREATE_PLATFORM_OPTIMIZED)
   set (OPENROX_USE_NEON true)
   add_definitions("/DROX_USE_NEON")
endif()
#Remove specific MS C implementation warnings
add_definitions("/D_CRT_SECURE_NO_WARNINGS")

#Debug flags
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG "/DDEBUG" "/MP")

#Optimize for current platform
list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS "/D_CRT_SECURE_NO_WARNINGS" "/MP")

# OpenMP options and flags
if (OPENROX_HAS_OPENMP_SUPPORT AND ${PROJECT_NAME}_USES_OPENMP)
   FIND_PACKAGE(OpenMP)
   if(OPENMP_FOUND)
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS         ${OpenMP_C_FLAGS})
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG   ${OpenMP_C_FLAGS_DEBUG})
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS       ${OpenMP_CXX_FLAGS})
      list(APPEND ${PROJECT_NAME}_LOCAL_CMAKE_CXX_FLAGS_DEBUG ${OpenMP_CXX_FLAGS_DEBUG})

      add_definitions("-DROX_USES_OPENMP")

      message("openmp found, adding openmp in ${PROJECT_NAME}")

   else()
      message("openmp NOT found, cannot be added in ${PROJECT_NAME}" )
   endif()
# else()
#    message("NOT adding openmp in ${PROJECT_NAME}" )
endif()

#Create argument lists for debug
foreach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG})
   set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${param}")
   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${param}")
endforeach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS_DEBUG})

#Create argument lists for release
foreach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS})
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${param}")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${param}")
endforeach(param ${${PROJECT_NAME}_LOCAL_CMAKE_C_FLAGS})

if (${PROJECT_NAME}_CREATE_SHARED_LIBRARIES)
   set (ROX_API "ROX_EXTERN_C ROX_EXPORT")
else()
   set(ROX_API "ROX_EXTERN_C")
endif()

