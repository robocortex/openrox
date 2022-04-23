//==============================================================================
//
//    OPENROX   : File neon.c
//
//    Contents  : Implementation of neon routines
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "neon.h"

#include <stdio.h>
#include <math.h>

#if defined (__arm__)
// Redefine vdivq_f32 for armv7
float32x4_t vdivq_f32(float32x4_t a, float32x4_t b)
{
   // get an initial estimate of 1/b.
   float32x4_t reciprocal = vrecpeq_f32(b);

   // use a couple Newton-Raphson steps to refine the estimate.  Depending on
   // accuracy requirements, it may be able to get away with only
   // one refinement (instead of the two used here).
   reciprocal = vmulq_f32(vrecpsq_f32(b, reciprocal), reciprocal);
   reciprocal = vmulq_f32(vrecpsq_f32(b, reciprocal), reciprocal);

   // and finally, compute a/b = a*(1/b)
   return vmulq_f32(a, reciprocal);
}
#endif


float32x4_t rox_f32_movehl_ps(float32x4_t __A, float32x4_t __B)
{
   float32x2_t a32 = vget_high_f32(vreinterpretq_u32_f32(__A));
   float32x2_t b32 = vget_high_f32(vreinterpretq_u32_f32(__B));
   return vreinterpretq_u32_f32(vcombine_f32(a32, b32));
}


float32x4_t rox_f32_add_ss(float32x4_t a, float32x4_t b)
{
   float32_t b0 = vgetq_lane_f32(vreinterpretq_u32_f32(b), 0);
   float32x4_t value = vsetq_lane_f32(b0, vdupq_n_f32(0), 0);
   // the upper values in the result must be the remnants of <a>.
   return vreinterpretq_u32_f32(vaddq_f32(a, value));
}


float rox_f32_hsum_ps(float32x4_t var)
{
   //float32x4_t shuf = vdupq_n_f32(var);      // broadcast elements 3,1 to 2,0
   //float32x4_t sums = vaddq_f32(var, shuf);
   //shuf = rox_f32_movehl_ps(shuf, sums); // high half -> low half
   //sums = rox_f32_add_ss(sums, shuf);
   //return  vgetq_lane_f32(vreinterpretq_u32_f32(a), 0);
   float32x4_t neon = vdupq_n_f32(0.0);
   return  vgetq_lane_f32(vreinterpretq_u32_f32(neon), 0);
}

int rox_f32_cmplt_or(float32x4_t var, float32x4_t min)
{
   float32x4_t var_is_lower_than_min = vreinterpretq_u32_f32(vcltq_f32(vreinterpretq_u32_f32(var), vreinterpretq_u32_f32(min)));
   float sum = rox_f32_hsum_ps(var_is_lower_than_min);
   if (fabsf(sum) <= 1e-5f)
   {
      return 0;
   }
   return 1;
}