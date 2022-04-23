#iOS SDK toolchain

#Set cmake system
set (CMAKE_SYSTEM_NAME Darwin)
set (UNIX 1)
set (APPLE 1)
set (IPHONEOS 1)
set (IOS 1)

# Use only arm architecture
set( ARCH "arm" )

set (CMAKE_OSX_DEPLOYMENT_TARGET "" CACHE STRING "Force unset of the deployment target for iOS" FORCE)

#Check System version for host
string (REGEX REPLACE "^([0-9]+)\\.([0-9]+).*$" "\\1" DARWIN_MAJOR_VERSION "${CMAKE_SYSTEM_VERSION}")
string (REGEX REPLACE "^([0-9]+)\\.([0-9]+).*$" "\\2" DARWIN_MINOR_VERSION "${CMAKE_SYSTEM_VERSION}")

#FIND ROOT OPEN
set (OPENROX_DEVELOPER_ROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer")
set (OPENROX_TOOLCHAIN_ROOT "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain")
if (NOT EXISTS ${OPENROX_DEVELOPER_ROOT})
message(FATAL_ERROR "Impossible to find ${OPENROX_DEVELOPER_ROOT}")
endif (NOT EXISTS ${OPENROX_DEVELOPER_ROOT})

#List available sdks
file (GLOB OPENROX_IOS_SDK_ROOTS "${OPENROX_DEVELOPER_ROOT}/SDKs/*")
if (NOT OPENROX_IOS_SDK_ROOTS)
message(FATAL_ERROR "No available sdk")
endif (NOT OPENROX_IOS_SDK_ROOTS)

#select greatest number sdk
list(SORT OPENROX_IOS_SDK_ROOTS)
list(REVERSE OPENROX_IOS_SDK_ROOTS)
list(GET OPENROX_IOS_SDK_ROOTS 0 OPENROX_IOS_SDK_ROOT)

include (CMakeForceCompiler)
CMAKE_FORCE_C_COMPILER (clang Clang)
CMAKE_FORCE_CXX_COMPILER (clang++ Clang)
set (CMAKE_AR ar CACHE FILEPATH "" FORCE)
#set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvm.clang.3_0")
set (CMAKE_CXX_COMPILER_WORKS TRUE)
set (CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set (CMAKE_OSX_ARCHITECTURES armv7 arm64)
set (CMAKE_OSX_SYSROOT ${OPENROX_IOS_SDK_ROOT})
set (CMAKE_IOS_DEVELOPER_ROOT ${OPENROX_DEVELOPER_ROOT})

set (CMAKE_FIND_ROOT_PATH ${OPENROX_DEVELOPER_ROOT} ${OPENROX_IOS_SDK_ROOT} ${OPENROX_TOOLCHAIN_ROOT} ${CMAKE_PREFIX_PATH})

set (CMAKE_FIND_FRAMEWORK FIRST)
set (CMAKE_SYSTEM_FRAMEWORK_PATH
		${OPENROX_IOS_SDK_ROOT}/System/Library/Frameworks
		${OPENROX_IOS_SDK_ROOT}/System/Library/PrivateFrameworks
		${OPENROX_IOS_SDK_ROOT}/Developer/Library/Frameworks)

add_definitions("-DROX_IS_IOS")
