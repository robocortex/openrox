#LINUX SPECIFIC

MESSAGE(STATUS "\n_____ rox_open/cmake/platform/platform_linux.cmake ____________________________________________\n" )

#set perplatform flags
set ( TARGET_PLATFORM               "linux")
set ( OPENROX_IS_LINUX                  true   )
set ( OPENROX_HAS_GUI_SUPPORT           true   )
set ( OPENROX_HAS_MOVIE_SUPPORT         true   )
set ( OPENROX_HAS_CAMERA_SUPPORT        true   )
set ( OPENROX_HAS_STD_EXAMPLES_SUPPORT  true   )
set ( OPENROX_HAS_STD_TESTS_SUPPORT     true   )
set ( OPENROX_HAS_MAC_ADDR_SUPPORT      true   )
set ( OPENROX_HAS_SHAREDSTATIC_SUPPORT  "both" )
set ( OPENROX_HAS_OBJMODEL_SUPPORT      true   )
set ( OPENROX_HAS_OPENMP_SUPPORT        true   )
set ( OPENROX_HAS_GPROF_SUPPORT         true   )

if ( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "^arm.*" )
   set ( OPENROX_HAS_NEON_SUPPORT true  )
   set ( OPENROX_HAS_SSE_SUPPORT  false )
   set ( OPENROX_HAS_AVX_SUPPORT  false )
else ()
   set ( OPENROX_HAS_NEON_SUPPORT false )
   set ( OPENROX_HAS_SSE_SUPPORT  true )
   set ( OPENROX_HAS_AVX_SUPPORT  true )
endif()

message ("OPENROX_HAS_NEON_SUPPORT : "  ${OPENROX_HAS_NEON_SUPPORT})
message ("OPENROX_HAS_SSE_SUPPORT : "   ${OPENROX_HAS_SSE_SUPPORT})
message ("OPENROX_HAS_AVX_SUPPORT : "   ${OPENROX_HAS_AVX_SUPPORT})

#define libraries for license check
list( APPEND OPENROX_LICENCE_LIBS "" )

if ( ${HAS_OPENGL} )
   list( APPEND OPENROX_EXTERNAL_LIBS "GL"       )
   list( APPEND OPENROX_EXTERNAL_LIBS "X11"      )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xinerama" )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xcursor"  )
   list( APPEND OPENROX_EXTERNAL_LIBS "rt"       )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xrandr"   )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xi"       )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xrender"  )
   list( APPEND OPENROX_EXTERNAL_LIBS "drm"      )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xdamage"  )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xxf86vm"  )
   list( APPEND OPENROX_EXTERNAL_LIBS "Xext"     )
endif()

list( APPEND OPENROX_EXTERNAL_LIBS "m"        )
list( APPEND OPENROX_EXTERNAL_LIBS "pthread"  )

# expose API version (ubuntu)
set (TARGET_API "16.04" CACHE STRING "Targetted API version of platform")
set_property(CACHE TARGET_API PROPERTY STRINGS "14.04;16.04" )

add_definitions("-DROX_IS_LINUX")
