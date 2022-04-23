#LINUX SPECIFIC

#set per-platform flags
set (OPENROX_IS_PIKEOS                  true  )
set (OPENROX_HAS_VIEWER_SUPPORT         false )
set (OPENROX_HAS_MOVIE_SUPPORT          false )
set (OPENROX_HAS_CAMERA_SUPPORT         false )
set (OPENROX_HAS_STD_EXAMPLES_SUPPORT   false )
set (OPENROX_HAS_STD_TESTS_SUPPORT      false )
set (OPENROX_HAS_MAC_ADDR_SUPPORT       false )
set (OPENROX_HAS_SHAREDSTATIC_SUPPORT   "both")
set (OPENROX_HAS_OBJMODEL_SUPPORT       false )
set (OPENROX_HAS_OPENMP_SUPPORT         false )
set (OPENROX_HAS_NEON_SUPPORT           false )
set (OPENROX_HAS_SSE_SUPPORT            false )

set (TOOLCHAIN_SETS_CMAKE_FLAGS         true  )

#define libraries for license check
list(APPEND OEPNROX_LICENCE_LIBS        "")
list(APPEND OPENROX_EXTERNAL_LIBS       "")
