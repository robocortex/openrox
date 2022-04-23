//==============================================================================
//
//    OPENROX   : File platform_avx.h
//
//    Contents  : API of platform_avx module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLATFORM_AVX__
#define __OPENROX_PLATFORM_AVX__

//! For AVX instructions, memory is vector aligned and data is padded to be a modulo of 32 bytes
#define ROX_DEFAULT_ALIGNMENT 32
#define ROX_ROW_BYTECOUNT_MULTIPLIER 32

//! Define used includes
#if defined(_MSC_VER)
#include <nmmintrin.h>
#include <immintrin.h>
#endif
#if defined(_MSC_VER) //// defined(__clang__)
#include <smmintrin.h>
#include <immintrin.h>
#else
#include <immintrin.h>
#endif

//! \brief Direct 8 float access
union avx_vector
{
   //! Tab member
   float tab[8];
   //! AVX member
   __m256 avx;
};

//! \brief Direct 32 char access
union avxi_vector
{
   //! Tab member
   unsigned char tab[32];
   //! AVX member
   __m256i avxi;
};

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
