//==============================================================================
//
//    OPENROX   : File sse_interaction_row_texture_matse3_model3d_zi.c
//
//    Contents  : Implementation of sse_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sse_interaction_row_texture_matse3_model3d_zi.h"
#include <float.h>
#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sse_interaction_row_texture_matse3_model3d_zi (
   __m128 * sse_Lk,
   const __m128 sse_ur,
   const __m128 sse_vr,
   const __m128 sse_Iu,
   const __m128 sse_Iv,
   const __m128 sse_zi,
   const __m128 sse_ziu,
   const __m128 sse_ziv,
   const __m128 sse_fu,
   const __m128 sse_fv,
   const __m128 sse_cu,
   const __m128 sse_cv,
   const __m128 sse_tau1,
   const __m128 sse_tau2,
   const __m128 sse_tau3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   __m128 sse_0 = _mm_set_ps1(0);
   __m128 sse_1 = _mm_set_ps1(1);
   __m128 sse_flt_min = _mm_set_ps1(FLT_MIN);

   __m128 sse_xr = _mm_div_ps(_mm_sub_ps(sse_ur, sse_cu), sse_fu);
   __m128 sse_yr = _mm_div_ps(_mm_sub_ps(sse_vr, sse_cv), sse_fv); 

   __m128 sse_a1 = _mm_sub_ps(_mm_mul_ps(sse_tau3, sse_xr), sse_tau1);
   __m128 sse_a2 = _mm_sub_ps(_mm_mul_ps(sse_tau3, sse_yr), sse_tau2);

   __m128 sse_d1 = _mm_mul_ps(sse_fu, sse_ziu);
   __m128 sse_d2 = _mm_mul_ps(sse_fv, sse_ziv);

   __m128 sse_s0 = _mm_sub_ps(sse_1, _mm_mul_ps(sse_tau3, sse_zi));

   __m128 sse_abs_s0 = rox_mm128_abs_ps(sse_s0);

   if (rox_mm128_cmplt_or(sse_abs_s0, sse_flt_min))
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); }
   
   __m128 sse_s0i = _mm_div_ps(sse_1, sse_s0);

   __m128 sse_xw = _mm_mul_ps(_mm_sub_ps(sse_xr, _mm_mul_ps(sse_zi, sse_tau1)), sse_s0i);
   __m128 sse_yw = _mm_mul_ps(_mm_sub_ps(sse_yr, _mm_mul_ps(sse_zi, sse_tau2)), sse_s0i);

   __m128 sse_s1 = _mm_mul_ps(sse_a1, sse_d1);
   __m128 sse_s2 = _mm_mul_ps(sse_a2, sse_d2);
   __m128 sse_sum_s0_s1_s2 = _mm_add_ps(_mm_add_ps(sse_s0, sse_s1), sse_s2);

   __m128 sse_abs_sum_s0_s1_s2 = rox_mm128_abs_ps(sse_sum_s0_s1_s2);
   
   if (rox_mm128_cmplt_or(sse_abs_sum_s0_s1_s2, sse_flt_min))
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); }

   __m128 sse_s = _mm_div_ps(sse_1, sse_sum_s0_s1_s2);

   __m128 sse_Iu_fus = _mm_mul_ps(sse_Iu, _mm_mul_ps(sse_fu, sse_s));
   __m128 sse_Iv_fvs = _mm_mul_ps(sse_Iv, _mm_mul_ps(sse_fv, sse_s));

   __m128 sse_Lpi_11 = _mm_add_ps(sse_s0, sse_s2);
   __m128 sse_Lpi_12 = _mm_mul_ps(_mm_sub_ps(sse_0, sse_a1), sse_d2);
   __m128 sse_Lpi_21 = _mm_mul_ps(_mm_sub_ps(sse_0, sse_a2), sse_d1);
   __m128 sse_Lpi_22 = _mm_add_ps(sse_s0, sse_s1);

   // __m128 sse_j_11 = sse_1;
   // __m128 sse_j_12 = sse_0;
   __m128 sse_j_13 = _mm_sub_ps(sse_0, sse_xw);

   __m128 sse_j_14 = _mm_mul_ps(_mm_sub_ps(sse_0, sse_yr), sse_xw);
   __m128 sse_j_15 = _mm_add_ps(sse_1, _mm_mul_ps(sse_xr, sse_xw));
   __m128 sse_j_16 = _mm_sub_ps(sse_0, sse_yr);

   // __m128 sse_j_21 = sse_0;
   // __m128 sse_j_22 = sse_1;
   __m128 sse_j_23 = _mm_sub_ps(sse_0, sse_yw);
   
   __m128 sse_j_24 = _mm_sub_ps(_mm_sub_ps(sse_0, sse_1), _mm_mul_ps(sse_yr, sse_yw));
   __m128 sse_j_25 = _mm_mul_ps(sse_xr, sse_yw);
   __m128 sse_j_26 = sse_xr;

   __m128 sse_vx = _mm_add_ps(_mm_mul_ps(sse_Iu_fus, sse_Lpi_11), _mm_mul_ps(sse_Iv_fvs, sse_Lpi_21));
   __m128 sse_vy = _mm_add_ps(_mm_mul_ps(sse_Iu_fus, sse_Lpi_12), _mm_mul_ps(sse_Iv_fvs, sse_Lpi_22));

   __m128 sse_vx_z = _mm_mul_ps(sse_vx, sse_zi);
   __m128 sse_vy_z = _mm_mul_ps(sse_vy, sse_zi);

   sse_Lk[0] = sse_vx_z;
   sse_Lk[1] = sse_vy_z;
   sse_Lk[2] = _mm_add_ps(_mm_mul_ps(sse_vx_z, sse_j_13), _mm_mul_ps(sse_vy_z, sse_j_23));
   sse_Lk[3] = _mm_add_ps(_mm_mul_ps(sse_vx,   sse_j_14), _mm_mul_ps(sse_vy,   sse_j_24));
   sse_Lk[4] = _mm_add_ps(_mm_mul_ps(sse_vx,   sse_j_15), _mm_mul_ps(sse_vy,   sse_j_25));
   sse_Lk[5] = _mm_add_ps(_mm_mul_ps(sse_vx,   sse_j_16), _mm_mul_ps(sse_vy,   sse_j_26));

   //ROX_UNUSED(sse_j_11);
   //ROX_UNUSED(sse_j_12);
   //ROX_UNUSED(sse_j_22);
   //ROX_UNUSED(sse_j_21);

function_terminate:
   return error;
}
