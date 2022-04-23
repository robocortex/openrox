
set(CMAKE_GENERATOR_TOOLSET DefaultClang)
set(CMAKE_SYSTEM_NAME Android)

SET(UNIX 1)
set(ANDROID 1)

unset(CMAKE_CXX_FLAGS CACHE)
unset(CMAKE_C_FLAGS CACHE)
unset(CMAKE_LINKER_FLAGS CACHE)

# Cache flags to prevent reset during cmake initialization
set( CMAKE_CXX_FLAGS "" CACHE STRING "c++ flags" )
set( CMAKE_C_FLAGS   ""  CACHE STRING "c flags" )
set( CMAKE_EXE_LINKER_FLAGS "" CACHE STRING "linker flags")

add_definitions("-DROX_IS_ANDROID -DANDROID")

# Define possible android APIs
SET(ANDROID_API "26" CACHE STRING "the android api to use, recommend 26")
SET_PROPERTY(CACHE ANDROID_API PROPERTY STRINGS "19;21;24;26" )

SET(ANDROID_VERSION 4.9)

# Define possible android target
SET(ANDROID_ARCH "arm64-v8a" CACHE STRING "Supported android targets.")
SET_PROPERTY(CACHE ANDROID_ARCH PROPERTY STRINGS "x86;armv7-a;arm64-v8a" )

if(${ANDROID_ARCH} MATCHES "x86")
   set(ANDROID_ABI x86)
   set(ARCH x86)
   set(ANDROID_TOOLCHAIN llvm/)
   set(ANDROID_PREFIX i686-linux-android)
elseif(${ANDROID_ARCH} MATCHES "arm64-v8a")
   set(ANDROID_ABI arm64-v8a)
   set(ARCH arm64)
   set(ANDROID_TOOLCHAIN llvm)
   set(ANDROID_PREFIX aarch64-linux-androideabi)
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__aarch64__")
else()
   set(ANDROID_ABI armeabi-v7a)
   set(ARCH arm)
   set(ANDROID_TOOLCHAIN llvm/)
   set(ANDROID_PREFIX arm-linux-androideabi)
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__arm__")
   #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfloat-abi=softfp")
   #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfloat-abi=softfp ")
endif()


# In recent versions, cmake is able to specify
# android min and target API
if( CMAKE_VERSION VERSION_GREATER 3.1.5 )
   set(CMAKE_ANDROID_API_MIN ${ANDROID_API})
   set(CMAKE_ANDROID_API ${ANDROID_API})
endif()

# Configure cmake android variable
set(CMAKE_ANDROID_ARCH ${ANDROID_ARCH})

# Not supported by current version of cmake (cmake 3.5.1)
if( CMAKE_VERSION VERSION_GREATER 3.7 )
  set(CMAKE_ANDROID_NDK_TOOLCHAIN_VERSION "4.9")
endif()

set(CMAKE_ANDROID_NDK "$ENV{OPENROX_ANDROID_HOME}")

unset(SDKROOT CACHE)
set(SDKROOT "${CMAKE_ANDROID_NDK}/platforms/android-${ANDROID_API}/arch-${ARCH}")

if(CMAKE_HOST_APPLE)
    set(HOST_SYSTEM_NAME "darwin-x86_64")
elseif(CMAKE_HOST_WIN32)
   set(CMAKE_C_USE_RESPONSE_FILE_FOR_OBJECTS   ON)
   set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_OBJECTS ON)
   set(CMAKE_EXECUTABLE_SUFFIX ".exe")
    set(HOST_SYSTEM_NAME "windows-x86_64")
    unset(WIN32)
else()
    set(HOST_SYSTEM_NAME "linux-x86_64")
endif()

# Force compiler
set(TOOLSROOT "${CMAKE_ANDROID_NDK}/toolchains/${ANDROID_TOOLCHAIN}/prebuilt/${HOST_SYSTEM_NAME}")
SET(CMAKE_C_COMPILER ${TOOLSROOT}/bin/clang${CMAKE_EXECUTABLE_SUFFIX} GNU)
SET(CMAKE_CXX_COMPILER ${TOOLSROOT}/bin/clang++${CMAKE_EXECUTABLE_SUFFIX} GNU)

# Force other programs
set(CMAKE_AR "${TOOLSROOT}/bin/${ANDROID_PREFIX}-ar${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "archive" FORCE)
set(CMAKE_CXX_COMPILER_AR "${TOOLSROOT}/bin/${ANDROID_PREFIX}-ar${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "archive" FORCE)
set(CMAKE_LINKER "${TOOLSROOT}/bin/${ANDROID_PREFIX}-ld${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "linker" FORCE)
set(CMAKE_NM "${TOOLSROOT}/bin/${ANDROID_PREFIX}-nm${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "nm" FORCE)
set(CMAKE_OBJCOPY "${TOOLSROOT}/bin/${ANDROID_PREFIX}-objcopy${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "objcopy" FORCE)
set(CMAKE_OBJDUMP "${TOOLSROOT}/bin/${ANDROID_PREFIX}-objdump${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "objdump" FORCE)
set(CMAKE_STRIP "${TOOLSROOT}/bin/${ANDROID_PREFIX}-strip${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "strip" FORCE)
set(CMAKE_RANLIB "${TOOLSROOT}/bin/${ANDROID_PREFIX}-ranlib${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "ranlib" FORCE)
set(CMAKE_CXX_COMPILER_RANLIB "${TOOLSROOT}/bin/${ANDROID_PREFIX}-ranlib${CMAKE_EXECUTABLE_SUFFIX}"  CACHE PATH "ranlib" FORCE)

if(NOT CMAKE_ANDROID_STL_TYPE)
  if("${CMAKE_GENERATOR}" MATCHES "Visual Studio ([0-9]+)")
    set(CMAKE_ANDROID_STL_TYPE llvm-libc++_static)
    set(ANDROID_STL_TYPE llvm-libc++)
  else()
    set(CMAKE_ANDROID_STL_TYPE gnustl_static)
    set(ANDROID_STL_TYPE gnu-libc++)
  endif()
endif()

# Path to STL library
if("${CMAKE_ANDROID_STL_TYPE}" MATCHES "llvm*")
    SET( libstl "${CMAKE_ANDROID_NDK}/sources/cxx-stl/${ANDROID_STL_TYPE}" )
else()
    SET( libstl "${CMAKE_ANDROID_NDK}/sources/cxx-stl/${ANDROID_STL_TYPE}/${ANDROID_VERSION}" )
endif()

SET( ANDROID_STL_INCLUDE_DIRS "${libstl}/include" "${libstl}/libs/${ANDROID_ABI}/include" "${libstl}/include/backward" ${CMAKE_ANDROID_NDK}/sources/android/support/include "${SDKROOT}/usr/include" )
SET( ANDROID_STL_LIB_DIRS ${libstl}/libs/${ANDROID_ABI} )

LINK_DIRECTORIES(${ANDROID_STL_LIB_DIRS})
INCLUDE_DIRECTORIES( ${ANDROID_STL_INCLUDE_DIRS} )

# Add flags for Android
if("${ANDROID_TOOLCHAIN}" MATCHES "llvm")
  INCLUDE_DIRECTORIES ( SYSTEM ${CMAKE_ANDROID_NDK}/sources/cxx-stl/llvm-libc++abi/include )
  set(CMAKE_LINKER_FLAGS   "${CMAKE_LINKER_FLAGS} -L${ANDROID_STL_LIB_DIRS}" CACHE STRING "executable linker flags" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__ARM_NEON -fexceptions -frtti --sysroot=${SDKROOT} -funwind-tables -fsigned-char -no-canonical-prefixes -fdata-sections -ffunction-sections")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__ARM_NEON -fexceptions --sysroot=${SDKROOT} -funwind-tables -fsigned-char -no-canonical-prefixes -fdata-sections -ffunction-sections")
else()
  set(CMAKE_LINKER_FLAGS   "${CMAKE_LINKER_FLAGS} -L${ANDROID_STL_LIB_DIRS} -D__ARM_NEON -Wl,-z,nocopyreloc -Wl,--fix-cortex-a9 -Wl,--no-undefined -Wl,--gc-sections -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now"  CACHE STRING "executable linker flags" )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__ARM_NEON -fexceptions -frtti -Wno-psabi --sysroot=${SDKROOT} -funwind-tables -finline-limit=64 -fsigned-char -no-canonical-prefixes -fdata-sections -ffunction-sections")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__ARM_NEON -fexceptions -Wno-psabi --sysroot=${SDKROOT} -funwind-tables -finline-limit=64 -fsigned-char -no-canonical-prefixes -fdata-sections -ffunction-sections")
endif()

set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -lgcc -latomic")
if("${ANDROID_TOOLCHAIN}" MATCHES "armeabi-v7a")
  set(CMAKE_LINKER_FLAGS   "${CMAKE_LINKER_FLAGS} -lunwind")
endif()

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_LINKER_FLAGS}")
SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${CMAKE_LINKER_FLAGS}")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_LINKER_FLAGS}")
SET(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} ${CMAKE_LINKER_FLAGS}")

# Strangly -02 does not seems to be handled anymore with android
if( CMAKE_VERSION VERSION_GREATER "3.14.0" )
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -O3")
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")
endif()

#file(COPY ${ANDROID_NDK}/prebuilt/android-arm/gdbserver/gdbserver DESTINATION ${PROJECT_SOURCE_DIR}/libs/${ANDROID_NDK_ABI_NAME}/)

# Define root path for cmake find functions
SET (CMAKE_FIND_ROOT_PATH "${SDKROOT}" "${TOOLSROOT}" )
SET (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
SET (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
