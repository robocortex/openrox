# ROX Core library definition
SET (OPENROX_CMAKE_DIR ${OPENROX_SOURCE_DIR}/cmake)
SET (OPENROX_CODE_DIR ${OPENROX_SOURCE_DIR}/sources)

# Define system include path
include_directories(${OPENROX_CODE_DIR})
include_directories(${OPENROX_SOURCE_DIR}/extern)

# API to include are generated in binary dir, force to use "#include <generated/file.h>
# include_directories(${OPENROX_BINARY_DIR}/generated)
include_directories(${OPENROX_BINARY_DIR})

# include_directories(${OPENROX_BINARY_DIR})

# Layer 1 : System files
# This layer is dedicated to all low level stuff (memory management, errors, etc).
include(${OPENROX_CMAKE_DIR}/layers/system_layer.cmake)

# Layer 2 : Baseproc files
# This layer is dedicated to all basic function which are not computer vision.
include(${OPENROX_CMAKE_DIR}/layers/baseproc_layer.cmake)

# Layer 3 : User files
# This layer is dedicated to all core metier functions/objects.
include(${OPENROX_CMAKE_DIR}/layers/core_layer.cmake)

# Layer 4 : User files
# This layer is dedicated to all high level functions and structures.
include(${OPENROX_CMAKE_DIR}/layers/user_layer.cmake)

# Layer 5 : IOLAYER files
# This layer is dedicated to all advanced input/output functions.
include(${OPENROX_CMAKE_DIR}/layers/io_layer.cmake)

# Layer 6 : Extern files
# This layer is dedicated to all external dependency functions.
include(${OPENROX_CMAKE_DIR}/layers/extern_layer.cmake)

# Stack all layers files together
set (OPENROX_SOURCES
    ${SYSTEM_LAYER_SOURCES}
    ${BASEPROC_LAYER_SOURCES}
    ${CORE_LAYER_SOURCES}
    ${USER_LAYER_SOURCES}
    ${IO_LAYER_SOURCES}
    ${EXTERN_LAYER_SOURCES}
)

# For convenience, sort files alphabetically (for fast find)
list(SORT OPENROX_SOURCES)

# Generate configuration file
configure_file(${SYSTEM_LAYER_SOURCES_DIR}/arch/config.h.in  ${OPENROX_BINARY_DIR}/generated/config.h)
configure_file(${OPENROX_CODE_DIR}/api/openrox.h ${OPENROX_BINARY_DIR}/generated/openrox.h)

if (OPENROX_BUILD_RELEASE)
   MESSAGE(STATUS "CREATE OPENROX RELEASE")
   SET(OPENROX_TARGET_RELEASE "openrox")
   configure_file(${OPENROX_CODE_DIR}/api/openrox.h ${OPENROX_BINARY_DIR}/generated/openrox.h)
else ()
   MESSAGE(STATUS "DO NOT CREATE OPENROX RELEASE BY DEFAULT")
   SET(OPENROX_TARGET_RELEASE "openrox")
   configure_file(${OPENROX_CODE_DIR}/api/openrox.h ${OPENROX_BINARY_DIR}/generated/openrox.h)
endif ()

# Create library as shared
if ( OPENROX_CREATE_SHARED_LIBRARIES )
   # Add external libs if this is a shared lib
   list (APPEND OPENROX_EXTERNAL_LIBS ${OPENROX_LICENCE_LIBS})
   if ( NOT OPENROX_BUILD_RELEASE )
      list (APPEND OPENROX_EXTERNAL_LIBS ${OPENROX_LINEAR_ALGEBRA_LIBS})
   endif()
   # list (APPEND OPENROX_EXTERNAL_LIBS ${ROX_OBJFILE_LIBS})
   # list (APPEND OPENROX_EXTERNAL_LIBS ${ROX_MODEL_LIBS})
   # list (APPEND OPENROX_EXTERNAL_LIBS ${ROX_VIZ_LIBS})
   # list (APPEND OPENROX_EXTERNAL_LIBS ${ROX_OPENCL_LIBS})
   # list (APPEND OPENROX_EXTERNAL_LIBS ${ROX_PNG_LIBS})

   add_library(openrox SHARED ${OPENROX_SOURCES})
   target_link_libraries(openrox ${OPENROX_EXTERNAL_LIBS})
   install_target(openrox)

   if(MSVC)
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:\"LIBCMT\"")
   endif()
else()
   add_library(openrox STATIC ${OPENROX_SOURCES})

   # set_target_properties(openrox PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")
   if (APPLE)
      #find_library(COREFOUNDATION_LIBRARY CoreFoundation)
      #find_library(IOKIT_LIBRARY IOKit)
      #target_link_libraries(openrox ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
      target_link_libraries(openrox ${OPENROX_LICENCE_LIBS})
   endif()
   if(WIN32)
   else()
      target_link_libraries(openrox m)
   endif()
endif()

if (OPENROX_BUILD_RELEASE)
   set_target_properties (openrox PROPERTIES OUTPUT_NAME ${OPENROX_TARGET_RELEASE})

   #if (OPENROX_IS_WINDOWS OR OPENROX_IS_UWP)
   #   option(OPENROX_BUILD_CSHARPWRAPPER "Build Csharp wrapper" OFF)
   #   if(OPENROX_BUILD_CSHARPWRAPPER)
   #      file(COPY "${OPENROX_SOURCE_DIR}/release/csharp/openrox_wrapper/" DESTINATION  ${OPENROX_BINARY_DIR}/openrox_wrapper)
   #      include_external_msproject(openrox_csharp "${OPENROX_BINARY_DIR}/openrox_wrapper/openrox_csharp.csproj" TYPE FAE04EC0-301F-11D3-BF4B-00C04F79EFBC)
   #   endif()
   # endif()
endif()

message("external libs: " ${OPENROX_EXTERNAL_LIBS})

# Documentation generation
include(manual/prog)
include(manual/user)
