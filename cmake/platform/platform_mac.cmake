#APPLE MACOS SPECIFIC

#set perplatform flags
set (TARGET_PLATFORM               "macosx"  )
set ( OPENROX_IS_MACOSX                true      )
set ( OPENROX_HAS_GUI_SUPPORT          true      )
set ( OPENROX_HAS_MOVIE_SUPPORT        true      )
set ( OPENROX_HAS_CAMERA_SUPPORT       false     )
set ( OPENROX_HAS_SSE_SUPPORT          true      )
set ( OPENROX_HAS_NEON_SUPPORT         false     )
set ( OPENROX_HAS_STD_EXAMPLES_SUPPORT true      )
set ( OPENROX_HAS_STD_TESTS_SUPPORT    true      )
set ( OPENROX_HAS_MAC_ADDR_SUPPORT     true      )
set ( OPENROX_HAS_SHAREDSTATIC_SUPPORT "both"    )
set ( OPENROX_HAS_OBJMODEL_SUPPORT     true      )

#define libraries for license check
list( APPEND OPENROX_LICENCE_LIBS      "-framework CoreFoundation -framework IOKit" )
list( APPEND OPENROX_EXTERNAL_LIBS ""                                           )

# expose API version (yosemite)
set (TARGET_API "10.10.5" CACHE STRING "Targetted API version of platform")

add_definitions("-DROX_IS_MACOSX")