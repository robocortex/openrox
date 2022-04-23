//==============================================================================
//
//    OPENROX   : File ssh.c
//
//    Contents  : Implementation of ssh module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sse.h"

#include <stdio.h>
#include <math.h>
#include "inout/system/print.h"

float rox_mm128_hsum_ps ( __m128 var ) 
{
    __m128 shuf = _mm_movehdup_ps(var);      // broadcast elements 3,1 to 2,0
    __m128 sums = _mm_add_ps(var, shuf);
    shuf        = _mm_movehl_ps(shuf, sums); // high half -> low half
    sums        = _mm_add_ss(sums, shuf);
    return        _mm_cvtss_f32(sums);
}

void rox_mm128_printf_float(__m128 var)
{
    float *val = (float*) &var; 
    rox_log("Numerical: %f %f %f %f \n", val[3], val[2], val[1], val[0] );
}

void rox_mm128i_printf_uint16 ( __m128i var )
{
    uint16_t *val = (uint16_t*) &var;
    rox_log("Numerical: %i %i %i %i %i %i %i %i \n", val[7], val[6], val[5], val[4], val[3], val[2], val[1], val[0] );
}

void rox_mm128i_printf_uint8 ( __m128i var )
{
    uint8_t *val = (uint8_t*) &var;
    printf("Numerical: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u \n", val[15], val[14], val[13], val[12], val[11], val[10], val[9], val[8], val[7], val[6], val[5], val[4], val[3], val[2], val[1], val[0] );
}

__m128 rox_mm128_abs_ps(__m128 var)
{
   __m128 sse_abs_mask = _mm_set1_ps(-0.0f);
   return _mm_andnot_ps(sse_abs_mask, var);
}

bool rox_mm128_cmplt_or(__m128 var, __m128 min)
{
   __m128 var_is_lower_than_min = _mm_cmplt_ps(var, min);
   float sum = rox_mm128_hsum_ps(var_is_lower_than_min);
   if (fabsf(sum) <= 1e-5f)
   {
      return false;
   }
   return true;
}

void rox_mm128i_loadu_16_uchar_to_vectors_8_short ( __m128i * vector_8_short_1, __m128i * vector_8_short_2, unsigned char * data_16_uchar )
{
   // Load 16 uchar into a 128 bit register (128 = 16*8)
   __m128i vector_16_uchar = _mm_loadu_si128((__m128i *) data_16_uchar);

   // Convert first 8 uchar (8bits) to 8 short (16 bit) in the first vector
   *vector_8_short_1 = _mm_cvtepu8_epi16(vector_16_uchar);
   // Convert other 8 uchar (8bits) to 8 short (16 bit) in the other vector
   *vector_8_short_2 = _mm_cvtepu8_epi16(_mm_srli_si128(vector_16_uchar, 8));
}