#ANDROID SPECIFIC

#set perplatform flags
set (TARGET_PLATFORM               "android")
set ( OPENROX_IS_ANDROID               true     )
set ( OPENROX_HAS_GUI_SUPPORT          false    )
set ( OPENROX_HAS_MOVIE_SUPPORT        false    )
set ( OPENROX_HAS_CAMERA_SUPPORT       false    )
set ( OPENROX_HAS_STD_EXAMPLES_SUPPORT false    )
set ( OPENROX_HAS_STD_TESTS_SUPPORT    true     )
set ( OPENROX_HAS_MAC_ADDR_SUPPORT     false    )
set ( OPENROX_HAS_SHAREDSTATIC_SUPPORT "shared" )
set ( OPENROX_HAS_OBJMODEL_SUPPORT     false    )
set ( OPENROX_IS_MOBILE                true     )

# x86 architectures does not support neon but sse
if(${CMAKE_ANDROID_ARCH} MATCHES x86)
   set ( OPENROX_HAS_NEON_SUPPORT      false    )
   set ( OPENROX_HAS_SSE_SUPPORT       false     ) # SSE is supported, but it currently does not compile
else()
   set ( OPENROX_HAS_NEON_SUPPORT      true     )
   set ( OPENROX_HAS_SSE_SUPPORT       false    )
endif()

#define libraries for license check
list( APPEND OPENROX_LICENCE_LIBS      ""  )
list( APPEND OPENROX_EXTERNAL_LIBS     "m" )

# expose API version
set (TARGET_API ${ANDROID_API})