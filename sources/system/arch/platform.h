//==============================================================================
//
//    OPENROX   : File platform.h
//
//    Contents  : API of platform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// Define platform specific information

//! By default use ansi platform
#ifdef ROX_USE_AVX
   #include "platform_avx.h"
#elif ROX_USE_SSE
   #include "platform_sse.h"
#elif ROX_USE_NEON
   #include "platform_neon.h"
#else
   #include "platform_ansi.h"
#endif

