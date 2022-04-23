//==============================================================================
//
//    OPENROX   : File ansi_linsys_texture_matse3_light_affine_model3d_zi.c
//
//    Contents  : Implementation of ansi_linsys_texture_matse3_light_affine_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_texture_matse3_light_affine_model3d_zi.h"

#include <stdio.h>

#include <system/vectorisation/neon.h>
#include <baseproc/calculus/jacobians/neon_interaction_row_texture_matse3_model3d_zi.h>

int rox_ansi_linsys_texture_matse3_light_affine_model3d_zi (
   double ** LtL_data,
   double ** Lte_data,
   double ** K_data,
   double ** tau_data,
   float ** Zi_data,
   float ** Ziu_data,
   float ** Ziv_data,
   float ** Iu_data,
   float ** Iv_data,
   float ** Id_data,
   float ** Ia_data,
   unsigned int ** Im_data,
   int rows,
   int cols
)
{
   int error = 0;

   float32x4_t neon_fu = vdupq_n_f32((float)K_data[0][0]);
   float32x4_t neon_fv = vdupq_n_f32((float)K_data[1][1]);
   float32x4_t neon_cu = vdupq_n_f32((float)K_data[0][2]);
   float32x4_t neon_cv = vdupq_n_f32((float)K_data[1][2]);

   float32x4_t neon_tau1 = vdupq_n_f32((float)tau_data[0][0]);
   float32x4_t neon_tau2 = vdupq_n_f32((float)tau_data[1][0]);
   float32x4_t neon_tau3 = vdupq_n_f32((float)tau_data[2][0]);

   float32x4_t neon_1 = vdupq_n_f32(1.0);
   float32x4_t neon_4 = vdupq_n_f32(4.0);

   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   float32x4_t neon_cols = vdupq_n_f32((float)cols);

   float32x4_t neon_LtL[8][8], neon_Lte[8];

   for (int v = 0; v < rows; v++)
   {
      float vr = (float)v;
      float32x4_t neon_vr = vdupq_n_f32(vr);

      unsigned int * ptr_Im = Im_data[v];
      float * ptr_Iu = Iu_data[v];
      float * ptr_Iv = Iv_data[v];
      float * ptr_Id = Id_data[v];
      float * ptr_Ia = Ia_data[v];

      float * ptr_Zi = Zi_data[v];
      float * ptr_Ziu = Ziu_data[v];
      float * ptr_Ziv = Ziv_data[v];

      Rox_Neon_Float uneon_ur;

      uneon_ur.tab[0] = 0;
      uneon_ur.tab[1] = 1;
      uneon_ur.tab[2] = 2;
      uneon_ur.tab[3] = 3;

      float32x4_t neon_ur = uneon_ur.ssetype;

      // Set avx_LtL and avx_Lte to 0
      for (Rox_Sint k = 0; k < 8; k++)
      {
         for (Rox_Sint l = 0; l <= k; l++)
         {
            neon_LtL[k][l] = vdupq_n_f32(0.0);
         }
         neon_Lte[k] = vdupq_n_f32(0.0);
      }

      for (int u = 0; u < cols4; u++)
      {
         uint32x4_t neon_Im = vld1q_u32(ptr_Im);
         neon_Im = vandq_u32(neon_Im, vcltq_f32(neon_ur, neon_cols));

         // unsigned int mask = 1; // _mm_movemask_ps(neon_Im);

         // // rox_mm128_printf_float(neon_Im);

         // if (mask)
         // {
         float32x4_t neon_L_row[8];

         float32x4_t neon_Iu = vld1q_f32(ptr_Iu);
         float32x4_t neon_Iv = vld1q_f32(ptr_Iv);
         float32x4_t neon_Id = vld1q_f32(ptr_Id);
         float32x4_t neon_Ia = vld1q_f32(ptr_Ia);
         float32x4_t neon_Zi = vld1q_f32(ptr_Zi);
         float32x4_t neon_Ziu = vld1q_f32(ptr_Ziu);
         float32x4_t neon_Ziv = vld1q_f32(ptr_Ziv);

         neon_Iu = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Iu), neon_Im));
         neon_Iv = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Iv), neon_Im));

         neon_Id = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Id), neon_Im));
         neon_Ia = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Ia), neon_Im));

         neon_Zi = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Zi), neon_Im));
         neon_Ziu = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Ziu), neon_Im));
         neon_Ziv = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_Ziv), neon_Im));

         error = rox_neon_interaction_row_texture_matse3_model3d_zi(neon_L_row, neon_ur, neon_vr, neon_Iu, neon_Iv, neon_Zi, neon_Ziu, neon_Ziv, neon_fu, neon_fv, neon_cu, neon_cv, neon_tau1, neon_tau2, neon_tau3);
         if (error) { goto function_terminate; }

         // Interaction matrix for the light affine model
         neon_L_row[6] = neon_Ia;
         neon_L_row[7] = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(neon_1), neon_Im));

         // Update lower triangular part of the system
         for (int k = 0; k < 8; k++)
         {
            for (int l = 0; l <= k; l++)
            {
               neon_LtL[k][l] = vaddq_f32(neon_LtL[k][l], vmulq_f32(neon_L_row[k], neon_L_row[l]));
            }
            neon_Lte[k] = vaddq_f32(neon_Lte[k], vmulq_f32(neon_L_row[k], neon_Id));
         }
         // }

         ptr_Iu += 4;
         ptr_Iv += 4;
         ptr_Id += 4;
         ptr_Ia += 4;
         ptr_Im += 4;

         ptr_Zi += 4;
         ptr_Ziu += 4;
         ptr_Ziv += 4;

         neon_ur = vaddq_f32(neon_ur, neon_4);
      }

      // Transfer lower triangular part of the system from NEON to CPU
      for (int k = 0; k < 8; k++)
      {
         for (int l = 0; l <= k; l++)
         {
            //LtL_data[k][l] += rox_mm128_hsum_ps ( neon_LtL[k][l] );
            LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 0);
            LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 1);
            LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 2);
            LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 3);

         }
         //Lte_data[k][0] += rox_mm128_hsum_ps ( neon_Lte[k] );
         Lte_data[k][0] += vgetq_lane_f32(neon_Lte[k], 0);
         Lte_data[k][0] += vgetq_lane_f32(neon_Lte[k], 1);
         Lte_data[k][0] += vgetq_lane_f32(neon_Lte[k], 2);
         Lte_data[k][0] += vgetq_lane_f32(neon_Lte[k], 3);
      }
   }

function_terminate:
   return error;
}
