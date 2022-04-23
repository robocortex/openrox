//==============================================================================
//
//    OPENROX   : File platform_sse.h
//
//    Contents  : API of platform_sse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLATFORM_SSE2__
#define __OPENROX_PLATFORM_SSE2__

//! For SSE<5 instructions, memory is vector aligned and data is padded to be a modulo of 16 bytes
#define ROX_DEFAULT_ALIGNMENT 16
#define ROX_ROW_BYTECOUNT_MULTIPLIER 16

//! Define used includes
#if defined(_MSC_VER)
#include <nmmintrin.h>
#endif
#if defined(_MSC_VER) //// defined(__clang__)
#include <smmintrin.h>
#else
#include <immintrin.h>
#endif

//! \brief Direct four float access
union ssevector
{
	//! Tab member
   float tab[4];
	//! Sse member
   __m128 sse;
};

//! \brief Direct 16 char access
union sseivector
{
	//! Tab member
   unsigned char tab[16];
	//! Sse member
   __m128i sse;
};

#endif
