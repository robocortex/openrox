//==============================================================================
//
//    OPENROX   : File sraid_match_sse.c
//
//    Contents  : Implementation of sraid_match module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraid_match.h"

unsigned int rox_sraid_match ( unsigned short * feat1, unsigned short * feat2 )
{
   __m128i inc1 = _mm_setzero_si128();
   __m128i inc2 = _mm_setzero_si128();

   for ( int i = 0; i < 8; i++)
   {
      __m128i blockdesc_left1 = _mm_loadu_si128((__m128i*)feat1);
      __m128i blockdesc_right1 = _mm_loadu_si128((__m128i*)feat2);
      __m128i blockdesc_left2 = _mm_loadu_si128((__m128i*)(feat1 + 8));
      __m128i blockdesc_right2 = _mm_loadu_si128((__m128i*)(feat2 + 8));

      __m128i ssediff1 = _mm_sub_epi16(blockdesc_left1, blockdesc_right1);
      __m128i ssediff2 = _mm_sub_epi16(blockdesc_left2, blockdesc_right2);

      __m128i ssediff321 = _mm_madd_epi16(ssediff1, ssediff1);
      __m128i ssediff322 = _mm_madd_epi16(ssediff2, ssediff2);

      inc1 = _mm_add_epi32(inc1, ssediff321);
      inc2 = _mm_add_epi32(inc2, ssediff322);

      feat1 += 16;
      feat2 += 16;
   }

   __m128i inc = _mm_add_epi32(inc1, inc2);
   inc = _mm_hadd_epi32(inc, inc);
   inc = _mm_hadd_epi32(inc, inc);

   return _mm_extract_epi32(inc, 0);
}
