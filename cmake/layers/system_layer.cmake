#SYSTEM LAYER DEFINITION
SET (SYSTEM_LAYER_SOURCES_DIR ${OPENROX_CODE_DIR}/system)

#layer 0 : templates, define the preprocessing parameters
include(${OPENROX_CMAKE_DIR}/layers/templates/array2d_templates.cmake)
include(${OPENROX_CMAKE_DIR}/layers/templates/dllist_templates.cmake)
include(${OPENROX_CMAKE_DIR}/layers/templates/dynvec_templates.cmake)
include(${OPENROX_CMAKE_DIR}/layers/templates/objset_templates.cmake)

#Add sources

SET (SYSTEM_LAYER_BASE_SOURCES
   ${SYSTEM_LAYER_SOURCES_DIR}/memory/datatypes.c
   ${SYSTEM_LAYER_SOURCES_DIR}/memory/array.c
   ${SYSTEM_LAYER_SOURCES_DIR}/memory/array2d.c

   # Version
   ${SYSTEM_LAYER_SOURCES_DIR}/version/version.c

   # Generated array2D
   ${OPENROX_BINARY_DIR}/generated/array2d_uchar.c
   ${OPENROX_BINARY_DIR}/generated/array2d_uint.c
   ${OPENROX_BINARY_DIR}/generated/array2d_ushort.c
   ${OPENROX_BINARY_DIR}/generated/array2d_sshort.c
   ${OPENROX_BINARY_DIR}/generated/array2d_sint.c
   ${OPENROX_BINARY_DIR}/generated/array2d_slint.c
   ${OPENROX_BINARY_DIR}/generated/array2d_float.c
   ${OPENROX_BINARY_DIR}/generated/array2d_double.c

   # Generated Dynvec
   ${OPENROX_BINARY_DIR}/generated/dynvec_uint.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_sint.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_double.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_sparse_value.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_array2d.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_array2d_double.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_array2d_float.c
   ${OPENROX_BINARY_DIR}/generated/dynvec_array2d_uint.c

   # Generated objset
   ${OPENROX_BINARY_DIR}/generated/objset_imask.c
   ${OPENROX_BINARY_DIR}/generated/objset_array2d_float.c
   ${OPENROX_BINARY_DIR}/generated/objset_array2d_uint.c
   ${OPENROX_BINARY_DIR}/generated/objset_array2d_uchar.c
   ${OPENROX_BINARY_DIR}/generated/objset_array2d_double.c
   ${OPENROX_BINARY_DIR}/generated/objset_array2d_sshort.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_sint.c
   ${OPENROX_BINARY_DIR}/generated/objset_dynvec_sparse_value.c
)

# Vectorisation

  if (OPENROX_USE_AVX)
    message("USE AVX")
    SET ( SYSTEM_LAYER_AVX_SOURCES
      ${SYSTEM_LAYER_SOURCES_DIR}/vectorisation/avx.c
    )
  endif ()

  if (OPENROX_USE_SSE OR OPENROX_USE_AVX)
    message("USE SSE")
    SET ( SYSTEM_LAYER_SSE_SOURCES
      ${SYSTEM_LAYER_SOURCES_DIR}/vectorisation/sse.c
    )
  endif ()

  if (OPENROX_USE_NEON)
    message("USE NEON")
    SET ( SYSTEM_LAYER_NEON_SOURCES
      ${SYSTEM_LAYER_SOURCES_DIR}/vectorisation/neon.c
    )
  endif ()

#Add sources
SET (SYSTEM_LAYER_SOURCES
   ${SYSTEM_LAYER_BASE_SOURCES}
   ${SYSTEM_LAYER_AVX_SOURCES}
   ${SYSTEM_LAYER_SSE_SOURCES}
   ${SYSTEM_LAYER_NEON_SOURCES}
)

if (OPENROX_USES_MEMORY_POOL)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/memory/memory_pool.c)
else()
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/memory/memory.c)
endif ()

#Timer and date by platform
list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date.c)
if (OPENROX_IS_MACOSX)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_mac.c)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_mac.c)
elseif (OPENROX_IS_LINUX OR OPENROX_IS_PIKEOS)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_linux.c)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_linux.c)
elseif (OPENROX_IS_WINDOWS)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_win.c)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_win.c)
elseif (OPENROX_IS_IOS)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_mac.c)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_mac.c)
elseif (OPENROX_IS_ANDROID)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_linux.c)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_linux.c)
elseif (OPENROX_IS_WINDOWSPHONE)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_wp.c)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_wp.c)
elseif (OPENROX_IS_UWP)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/timer_wp.c)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/time/date_wp.c)
endif ()

#MacAddress by platform
list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address.c)
if (OPENROX_IS_MACOSX)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_mac.c)
elseif (OPENROX_IS_LINUX)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_linux.c)
elseif (OPENROX_IS_WINDOWS)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_win.c)
elseif (OPENROX_IS_IOS)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_ios.c)
elseif (OPENROX_IS_ANDROID)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_android.c)
elseif (OPENROX_IS_WINDOWSPHONE)
   list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_wp.c)
elseif (OPENROX_IS_UWP)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_wp.c)
elseif (OPENROX_IS_PIKEOS)
  list(APPEND SYSTEM_LAYER_SOURCES ${SYSTEM_LAYER_SOURCES_DIR}/network/mac_address_pikeos.c)
endif ()


#define hierarchical organization for ide projects
source_group("system" FILES ${SYSTEM_LAYER_SOURCES})
