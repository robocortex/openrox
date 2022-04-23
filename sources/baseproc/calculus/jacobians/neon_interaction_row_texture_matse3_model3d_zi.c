//==============================================================================
//
//    OPENROX   : File neon_interaction_row_texture_matse3_model3d_zi.c
//
//    Contents  : Implementation of neon_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "neon_interaction_row_texture_matse3_model3d_zi.h"

#include <math.h>
#include <float.h>

int rox_neon_interaction_row_texture_matse3_model3d_zi (
   float32x4_t * neon_Lk,
   const float32x4_t neon_ur,
   const float32x4_t neon_vr,
   const float32x4_t neon_Iu,
   const float32x4_t neon_Iv,
   const float32x4_t neon_zi,
   const float32x4_t neon_ziu,
   const float32x4_t neon_ziv,
   const float32x4_t neon_fu,
   const float32x4_t neon_fv,
   const float32x4_t neon_cu,
   const float32x4_t neon_cv,
   const float32x4_t neon_tau1,
   const float32x4_t neon_tau2,
   const float32x4_t neon_tau3
)
{
   int error = 0;

   float32x4_t neon_0 = vdupq_n_f32(0);
   float32x4_t neon_1 = vdupq_n_f32(1);

   float32x4_t neon_xr = vdivq_f32(vsubq_f32(neon_ur, neon_cu), neon_fu);
   float32x4_t neon_yr = vdivq_f32(vsubq_f32(neon_vr, neon_cv), neon_fv);

   float32x4_t neon_a1 = vsubq_f32(vmulq_f32(neon_tau3, neon_xr), neon_tau1);
   float32x4_t neon_a2 = vsubq_f32(vmulq_f32(neon_tau3, neon_yr), neon_tau2);

   float32x4_t neon_d1 = vmulq_f32(neon_fu, neon_ziu);
   float32x4_t neon_d2 = vmulq_f32(neon_fv, neon_ziv);

   float32x4_t neon_s0 = vsubq_f32(neon_1, vmulq_f32(neon_tau3, neon_zi));

   //float32x4_t neon_flt_min = vdupq_n_f32(FLT_MIN);
   //float32x4_t neon_abs_s0 = vabsq_f32(neon_s0);
   //if (rox_f32_cmplt_or(neon_abs_s0, neon_flt_min))
   //{ 
   //   error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); 
   //}

   float32x4_t neon_s0i = vdivq_f32(neon_1, neon_s0);

   float32x4_t neon_xw = vmulq_f32(vsubq_f32(neon_xr, vmulq_f32(neon_zi, neon_tau1)), neon_s0i);
   float32x4_t neon_yw = vmulq_f32(vsubq_f32(neon_yr, vmulq_f32(neon_zi, neon_tau2)), neon_s0i);

   float32x4_t neon_s1 = vmulq_f32(neon_a1, neon_d1);
   float32x4_t neon_s2 = vmulq_f32(neon_a2, neon_d2);
   float32x4_t neon_sum_s0_s1_s2 = vaddq_f32(vaddq_f32(neon_s0, neon_s1), neon_s2);

   //float32x4_t neon_abs_sum_s0_s1_s2 = vabsq_f32(neon_sum_s0_s1_s2);
   //if (rox_f32_cmplt_or(neon_abs_sum_s0_s1_s2, neon_flt_min))
   //{ 
   //   error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error);
   //}

   float32x4_t neon_s = vdivq_f32(neon_1, neon_sum_s0_s1_s2);

   float32x4_t neon_Iu_fus = vmulq_f32(neon_Iu, vmulq_f32(neon_fu, neon_s));
   float32x4_t neon_Iv_fvs = vmulq_f32(neon_Iv, vmulq_f32(neon_fv, neon_s));

   float32x4_t neon_Lpi_11 = vaddq_f32(neon_s0, neon_s2);
   float32x4_t neon_Lpi_12 = vmulq_f32(vsubq_f32(neon_0, neon_a1), neon_d2);
   float32x4_t neon_Lpi_21 = vmulq_f32(vsubq_f32(neon_0, neon_a2), neon_d1);
   float32x4_t neon_Lpi_22 = vaddq_f32(neon_s0, neon_s1);

   // float32x4_t neon_j_11 = neon_1;
   // float32x4_t neon_j_12 = neon_0;
   float32x4_t neon_j_13 = vsubq_f32(neon_0, neon_xw);

   float32x4_t neon_j_14 = vmulq_f32(vsubq_f32(neon_0, neon_yr), neon_xw);
   float32x4_t neon_j_15 = vaddq_f32(neon_1, vmulq_f32(neon_xr, neon_xw));
   float32x4_t neon_j_16 = vsubq_f32(neon_0, neon_yr);

   // float32x4_t neon_j_21 = neon_0;
   // float32x4_t neon_j_22 = neon_1;
   float32x4_t neon_j_23 = vsubq_f32(neon_0, neon_yw);

   float32x4_t neon_j_24 = vsubq_f32(vsubq_f32(neon_0, neon_1), vmulq_f32(neon_yr, neon_yw));
   float32x4_t neon_j_25 = vmulq_f32(neon_xr, neon_yw);
   float32x4_t neon_j_26 = neon_xr;

   float32x4_t neon_vx = vaddq_f32(vmulq_f32(neon_Iu_fus, neon_Lpi_11), vmulq_f32(neon_Iv_fvs, neon_Lpi_21));
   float32x4_t neon_vy = vaddq_f32(vmulq_f32(neon_Iu_fus, neon_Lpi_12), vmulq_f32(neon_Iv_fvs, neon_Lpi_22));

   float32x4_t neon_vx_z = vmulq_f32(neon_vx, neon_zi);
   float32x4_t neon_vy_z = vmulq_f32(neon_vy, neon_zi);

   neon_Lk[0] = neon_vx_z;
   neon_Lk[1] = neon_vy_z;
   neon_Lk[2] = vaddq_f32(vmulq_f32(neon_vx_z, neon_j_13), vmulq_f32(neon_vy_z, neon_j_23));
   neon_Lk[3] = vaddq_f32(vmulq_f32(neon_vx, neon_j_14), vmulq_f32(neon_vy, neon_j_24));
   neon_Lk[4] = vaddq_f32(vmulq_f32(neon_vx, neon_j_15), vmulq_f32(neon_vy, neon_j_25));
   neon_Lk[5] = vaddq_f32(vmulq_f32(neon_vx, neon_j_16), vmulq_f32(neon_vy, neon_j_26));

   //ROX_UNUSED(neon_j_11);
   //ROX_UNUSED(neon_j_12);
   //ROX_UNUSED(neon_j_22);
   //ROX_UNUSED(neon_j_21);

//function_terminate:
   return error;
}
