//==============================================================================
//
//    OPENROX   : File avx.c
//
//    Contents  : Implementation of avx tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "avx.h"
#include "sse.h"

#include <stdio.h>
#include <math.h>

float rox_mm256_hsum_ps(__m256 var) 
{
    __m128 vlow  = _mm256_castps256_ps128(var);
    __m128 vhigh = _mm256_extractf128_ps(var, 1); // high 128
           vlow  = _mm_add_ps(vlow, vhigh);     // add the low 128
    return rox_mm128_hsum_ps(vlow);         // and inline the sse3 version, which is optimal for AVX
    // (no wasted instructions, and all of them are the 4B minimum)
}

void rox_mm256_printf_float(__m256 var)
{
    float *val = (float*) &var; 
    printf("Numerical: %f %f %f %f %f %f %f %f \n", val[7], val[6], val[5], val[4], val[3], val[2], val[1], val[0] );
}

__m256 rox_mm256_abs_ps(__m256 var)
{
   __m256 sse_abs_mask = _mm256_set1_ps(-0.0f);
   return _mm256_andnot_ps(sse_abs_mask, var);
}

bool rox_mm256_cmplt_or(__m256 var, __m256 min)
{
   __m256 var_is_lower_than_min = _mm256_cmp_ps(var, min, _CMP_LT_OS);
   float sum = rox_mm256_hsum_ps(var_is_lower_than_min);
   if (fabsf(sum) <= 1e-5f)
   {
      return false;
   }
   return true;
}

void rox_mm256i_printf_uint16 ( __m256i var )
{
    unsigned int *val = (unsigned int *) &var; 
    printf("Numerical: %u %u %u %u %u %u %u %u \n", val[7], val[6], val[5], val[4], val[3], val[2], val[1], val[0] );
}