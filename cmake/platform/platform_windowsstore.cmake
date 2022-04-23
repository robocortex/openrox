#WINDOWS SPECIFIC

#set perplatform flags
set (OPENROX_IS_WINDOWS               false )
set (OPENROX_IS_UWP                   true  )
set (OPENROX_HAS_VIEWER_SUPPORT       true  )
set (OPENROX_HAS_MOVIE_SUPPORT        false )
set (OPENROX_HAS_CAMERA_SUPPORT       false )
set (OPENROX_HAS_SSE_SUPPORT          true  )
set (OPENROX_HAS_NEON_SUPPORT         false )
set (OPENROX_HAS_STD_EXAMPLES_SUPPORT true  )
set (OPENROX_HAS_STD_TESTS_SUPPORT    true  )
set (OPENROX_HAS_MAC_ADDR_SUPPORT     true  )
set (OPENROX_HAS_SHAREDSTATIC_SUPPORT "both")
set (OPENROX_HAS_OBJMODEL_SUPPORT     true  )
set (OPENROX_HAS_OPENMP_SUPPORT       true  )

#define libraries for license check
list(APPEND OPENROX_LICENCE_LIBS       "")
list(APPEND OPENROX_EXTERNAL_LIBS      "")

add_definitions("-DROX_IS_WINDOWS")