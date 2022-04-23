//==============================================================================
//
//    OPENROX   : File avx_interaction_row_texture_matse3_model3d_zi.c
//
//    Contents  : Implementation of avx_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "avx_interaction_row_texture_matse3_model3d_zi.h"
#include <float.h>
#include <inout/system/errors_print.h>
#include <system/errors/errors.h>

// This function should be in file "avx_interaction_row_texture_matse3_model3d_zi.c"
Rox_ErrorCode rox_avx_interaction_row_texture_matse3_model3d_zi (
   __m256 * avx_Lk,
   const __m256 avx_ur,
   const __m256 avx_vr,
   const __m256 avx_Iu,
   const __m256 avx_Iv,
   const __m256 avx_Zi,
   const __m256 avx_Ziu,
   const __m256 avx_Ziv,
   const __m256 avx_fu,
   const __m256 avx_fv,
   const __m256 avx_cu,
   const __m256 avx_cv,
   const __m256 avx_tau1,
   const __m256 avx_tau2,
   const __m256 avx_tau3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   __m256 avx_0 = _mm256_set1_ps(0);
   __m256 avx_1 = _mm256_set1_ps(1);
   __m256 avx_flt_min = _mm256_set1_ps(FLT_MIN);

   __m256 avx_xr = _mm256_div_ps(_mm256_sub_ps(avx_ur, avx_cu), avx_fu);
   __m256 avx_yr = _mm256_div_ps(_mm256_sub_ps(avx_vr, avx_cv), avx_fv); 
   __m256 avx_a1 = _mm256_sub_ps(_mm256_mul_ps(avx_tau3, avx_xr), avx_tau1);
   __m256 avx_a2 = _mm256_sub_ps(_mm256_mul_ps(avx_tau3, avx_yr), avx_tau2);

   __m256 avx_d1 = _mm256_mul_ps(avx_fu, avx_Ziu);
   __m256 avx_d2 = _mm256_mul_ps(avx_fv, avx_Ziv);

   __m256 avx_s0 = _mm256_sub_ps(avx_1, _mm256_mul_ps(avx_tau3, avx_Zi));

   // Compute absolute value
   // With juste the signed bit to one, used to mask absolute values
   __m256 avx_abs_s0 = rox_mm256_abs_ps(avx_s0);

   if (rox_mm256_cmplt_or(avx_abs_s0, avx_flt_min))
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); }

   __m256 avx_s0i = _mm256_div_ps(avx_1, avx_s0);

   __m256 avx_xw = _mm256_mul_ps(_mm256_sub_ps(avx_xr, _mm256_mul_ps(avx_Zi, avx_tau1)), avx_s0i);
   __m256 avx_yw = _mm256_mul_ps(_mm256_sub_ps(avx_yr, _mm256_mul_ps(avx_Zi, avx_tau2)), avx_s0i);

   __m256 avx_s1 = _mm256_mul_ps(avx_a1, avx_d1);
   __m256 avx_s2 = _mm256_mul_ps(avx_a2, avx_d2);
   __m256 avx_sum_s0_s1_s2 = _mm256_add_ps(_mm256_add_ps(avx_s0, avx_s1), avx_s2);

   // Compute absolute value
   // With just the signed bit to one, used to mask absolute values
   __m256 avx_abs_sum_s0_s1_s2 = rox_mm256_abs_ps(avx_sum_s0_s1_s2);
   
   if (rox_mm256_cmplt_or(avx_abs_sum_s0_s1_s2, avx_flt_min))
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); }

   __m256 avx_s = _mm256_div_ps(avx_1, avx_sum_s0_s1_s2);

   __m256 avx_Iu_fus = _mm256_mul_ps(avx_Iu, _mm256_mul_ps(avx_fu, avx_s));
   __m256 avx_Iv_fvs = _mm256_mul_ps(avx_Iv, _mm256_mul_ps(avx_fv, avx_s));

   __m256 avx_Lpi_11 = _mm256_add_ps(avx_s0, avx_s2);
   __m256 avx_Lpi_12 = _mm256_mul_ps(_mm256_sub_ps(avx_0, avx_a1), avx_d2);
   __m256 avx_Lpi_21 = _mm256_mul_ps(_mm256_sub_ps(avx_0, avx_a2), avx_d1);
   __m256 avx_Lpi_22 = _mm256_add_ps(avx_s0, avx_s1);

   // __m256 avx_j_11 = avx_1;
   // __m256 avx_j_12 = avx_0;
   __m256 avx_j_13 = _mm256_sub_ps(avx_0, avx_xw);

   __m256 avx_j_14 = _mm256_mul_ps(_mm256_sub_ps(avx_0, avx_yr), avx_xw);
   __m256 avx_j_15 = _mm256_add_ps(avx_1, _mm256_mul_ps(avx_xr, avx_xw));
   __m256 avx_j_16 = _mm256_sub_ps(avx_0, avx_yr);

   // __m256 avx_j_21 = avx_0;
   // __m256 avx_j_22 = avx_1;
   __m256 avx_j_23 = _mm256_sub_ps(avx_0, avx_yw);
   
   __m256 avx_j_24 = _mm256_sub_ps(_mm256_sub_ps(avx_0, avx_1), _mm256_mul_ps(avx_yr, avx_yw));
   __m256 avx_j_25 = _mm256_mul_ps(avx_xr, avx_yw);
   __m256 avx_j_26 = avx_xr;

   __m256 avx_vx = _mm256_add_ps(_mm256_mul_ps(avx_Iu_fus, avx_Lpi_11), _mm256_mul_ps(avx_Iv_fvs, avx_Lpi_21));
   __m256 avx_vy = _mm256_add_ps(_mm256_mul_ps(avx_Iu_fus, avx_Lpi_12), _mm256_mul_ps(avx_Iv_fvs, avx_Lpi_22));

   __m256 avx_vx_z = _mm256_mul_ps(avx_vx, avx_Zi);
   __m256 avx_vy_z = _mm256_mul_ps(avx_vy, avx_Zi);

   avx_Lk[0] = avx_vx_z;
   avx_Lk[1] = avx_vy_z;
   avx_Lk[2] = _mm256_add_ps(_mm256_mul_ps(avx_vx_z, avx_j_13), _mm256_mul_ps(avx_vy_z, avx_j_23));
   avx_Lk[3] = _mm256_add_ps(_mm256_mul_ps(avx_vx,   avx_j_14), _mm256_mul_ps(avx_vy,   avx_j_24));
   avx_Lk[4] = _mm256_add_ps(_mm256_mul_ps(avx_vx,   avx_j_15), _mm256_mul_ps(avx_vy,   avx_j_25));
   avx_Lk[5] = _mm256_add_ps(_mm256_mul_ps(avx_vx,   avx_j_16), _mm256_mul_ps(avx_vy,   avx_j_26));

   // ROX_UNUSED(avx_j_11);
   // ROX_UNUSED(avx_j_12);
   // ROX_UNUSED(avx_j_22);
   // ROX_UNUSED(avx_j_21);

function_terminate:
   return error;
}
