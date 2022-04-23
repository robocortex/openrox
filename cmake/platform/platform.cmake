#### PLATFORM INFORMATION ####

MESSAGE(STATUS "\n_____ rox_open/cmake/platform/platform.cmake ____________________________________________\n" )

MESSAGE(STATUS "sources in ${OPENROX_SOURCE_DIR}")


#Check the std types are compatible with the library on this platform
include( CheckTypeSize )

if ( ( NOT IOS ) AND ( NOT ANDROID ) AND ( NOT PIKEOS ) )
   set( CMAKE_EXTRA_INCLUDE_FILES )
   CHECK_TYPE_SIZE( "double" SDOUBLE )
   CHECK_TYPE_SIZE( "float" SFLOAT )
   CHECK_TYPE_SIZE( "char" SCHAR )
   CHECK_TYPE_SIZE( "short" SSHORT )
   CHECK_TYPE_SIZE( "int" SINT )
   if ( MSVC90 )
      CHECK_TYPE_SIZE( "__int64" SLONG )
   else()
      CHECK_TYPE_SIZE( "int64_t" SLONG )
   endif()
   CHECK_TYPE_SIZE( "long long int" SLLINT )
   set( CMAKE_EXTRA_INCLUDE_FILES )

   if ( NOT( ${SDOUBLE} EQUAL 8 ) )
     message( FATAL_ERROR "Incorrect double size" )
   endif( NOT( ${SDOUBLE} EQUAL 8 ) )

   if ( NOT( ${SFLOAT} EQUAL 4 ) )
     message( FATAL_ERROR "Incorrect float size" )
   endif( NOT( ${SFLOAT} EQUAL 4 ) )

   if ( NOT( ${SCHAR} EQUAL 1 ) )
     message( FATAL_ERROR "Incorrect char size" )
   endif( NOT( ${SCHAR} EQUAL 1 ) )

   if ( NOT( ${SSHORT} EQUAL 2 ) )
     message( FATAL_ERROR "Incorrect double size" )
   endif( NOT( ${SSHORT} EQUAL 2 ) )

   if ( NOT( ${SINT} EQUAL 4 ) )
     message( FATAL_ERROR "Incorrect int size" )
   endif( NOT( ${SINT} EQUAL 4 ) )

   if ( NOT( ${SLONG} EQUAL 8 ) )
     message( FATAL_ERROR "Incorrect long size" )
   endif( NOT( ${SLONG} EQUAL 8 ) )

   if ( NOT( ${SLLINT} EQUAL 8 ) )
     message( FATAL_ERROR "Incorrect long long int size" )
   endif( NOT( ${SLLINT} EQUAL 8 ) )
endif( ( NOT IOS ) AND ( NOT ANDROID ) AND ( NOT PIKEOS ) )

# Per platform configuration
if ( IOS )
   # MacOSX
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_ios.cmake )
elseif( ANDROID )
   # Android
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_android.cmake )
elseif(WINDOWSSTORE)
   # WindowsStore
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_windowsstore.cmake)
elseif( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
   # MacOSX
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_mac.cmake )
elseif( "${CMAKE_SYSTEM_NAME}" MATCHES "Linux" )
   MESSAGE ("System name is Linux")
   # Linux
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_linux.cmake )
elseif( "${CMAKE_SYSTEM_NAME}" MATCHES "Windows" )
   # Windows
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_windows.cmake )
elseif(PIKEOS)
   # PikeOS
   include( ${OPENROX_SOURCE_DIR}/cmake/platform/platform_pikeos.cmake)
else()
   #Unknown platform
   message( FATAL_ERROR "Unknown platform" )
endif()

# Detect 32 or 64 bits architecture
if (NOT OPENROX_IS_MOBILE )
  if ( CMAKE_SIZEOF_VOID_P EQUAL 4 )
     add_definitions( "-DROX_IS_32BITS" )
     set( ARCH "x86_32" )
  elseif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
     add_definitions( "-DROX_IS_64BITS" )
     set( ARCH "x86_64" )
  endif()
endif()
