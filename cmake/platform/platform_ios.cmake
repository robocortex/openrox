#APPLE MACOS SPECIFIC

#set perplatform flags
set (TARGET_PLATFORM                   "ios"     )
set ( OPENROX_IS_IOS                   true      )
set ( OPENROX_HAS_GUI_SUPPORT          false     )
set ( OPENROX_HAS_MOVIE_SUPPORT        false     )
set ( OPENROX_HAS_CAMERA_SUPPORT       false     )
set ( OPENROX_HAS_SSE_SUPPORT          false     )
set ( OPENROX_HAS_NEON_SUPPORT         true      )
set ( OPENROX_HAS_STD_EXAMPLES_SUPPORT false     )
set ( OPENROX_HAS_STD_TESTS_SUPPORT    false     )
set ( OPENROX_HAS_MAC_ADDR_SUPPORT     false     )
set ( OPENROX_HAS_SHAREDSTATIC_SUPPORT "static"  )
set ( OPENROX_HAS_OBJMODEL_SUPPORT     false     )
set ( OPENROX_IS_MOBILE                true      )

#define libraries for license check
list( APPEND OPENROX_LICENCE_LIBS      "" )
list( APPEND OPENROX_EXTERNAL_LIBS     "" )

# expose API version
set (TARGET_API "9.2.1" CACHE STRING "Targetted API version of platform")