#WINDOWS SPECIFIC

#set per-platform flags
set ( TARGET_PLATFORM               "windows")
set ( OPENROX_IS_WINDOWS                true     )
set ( OPENROX_HAS_GUI_SUPPORT           true     )
set ( OPENROX_HAS_MOVIE_SUPPORT         true     )
set ( OPENROX_HAS_CAMERA_SUPPORT        true     )
set ( OPENROX_HAS_STD_EXAMPLES_SUPPORT  true     )
set ( OPENROX_HAS_STD_TESTS_SUPPORT     true     )
set ( OPENROX_HAS_MAC_ADDR_SUPPORT      true     )
set ( OPENROX_HAS_SHAREDSTATIC_SUPPORT "both"    )
set ( OPENROX_HAS_OBJMODEL_SUPPORT      true     )
set ( OPENROX_HAS_OPENMP_SUPPORT        true     )
set ( OPENROX_HAS_SSE_SUPPORT           true     )
set ( OPENROX_HAS_AVX_SUPPORT           true     )
set ( OPENROX_HAS_NEON_SUPPORT          false    )

#define libraries for license check
list(APPEND OPENROX_LICENCE_LIBS "")
list(APPEND OPENROX_EXTERNAL_LIBS "")

# expose API version

set (TARGET_API "10" CACHE STRING "Targetted API version of platform")
set_property(CACHE TARGET_API PROPERTY STRINGS "xp;7;8.1;10" )

add_definitions("-DROX_IS_WINDOWS")
