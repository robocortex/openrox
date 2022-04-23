//==============================================================================
//
//    OPENROX   : File ansi_linsys_texture_matsl3_light_affine_neon.c
//
//    Contents  : Implementation of linsys_texture_matsl3_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_texture_matsl3_light_affine.h"

#include <system/vectorisation/neon.h>

int rox_ansi_linsys_texture_matsl3_light_affine (   
   double ** LtL_data, 
   double *  Lte_data,
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

   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   uint32x4_t neon_mask;
   
   float32x4_t neon_ur;
   float32x4_t neon_iu, neon_iv, neon_d, neon_a;
   float32x4_t neon_L_row[10], neon_LtL[10][10], neon_Lte[10];
   Rox_Neon_Float uneon_ur;

   float32x4_t neon_cols = vdupq_n_f32(cols);
   float32x4_t neon_0 = vdupq_n_f32(0);
   float32x4_t neon_1 = vdupq_n_f32(1);
   float32x4_t neon_4 = vdupq_n_f32(4);

   uneon_ur.tab[0] = 0;
   uneon_ur.tab[1] = 1;
   uneon_ur.tab[2] = 2;
   uneon_ur.tab[3] = 3;

   for (int k = 0; k < 10; k++)
   {
      for (int l = 0; l <= k; l++)
      {
         neon_LtL[k][l] = vdupq_n_f32(0.0);
      }

      neon_Lte[k] = vdupq_n_f32(0.0);
   }

   for (int i = 0; i < rows; i++)
   {
      double v = (double) i;
      float32x4_t neon_v = vdupq_n_f32(v);

      unsigned int  * ptrm = Im_data[i];
      float * ptrgx = Iu_data[i];
      float * ptrgy = Iv_data[i];
      float * ptrd = Id_data[i];
      float * ptra = Ia_data[i];

      neon_ur = uneon_ur.ssetype;

      for (int j = 0; j < cols4; j++)
      {
         // double u = (double) j;
         // float32x4_t neon_u = vdupq_n_f32(u);

         neon_iu = vld1q_f32(ptrgx);
         neon_iv = vld1q_f32(ptrgy);
         neon_d = vld1q_f32(ptrd);
         neon_a = vld1q_f32(ptra);

         neon_mask = vld1q_u32(ptrm);
         neon_mask = vandq_u32(neon_mask, vcltq_f32(neon_ur, neon_cols));

         neon_iu = vreinterpretq_f32_u32(vandq_u32(neon_mask, vreinterpretq_u32_f32(neon_iu)));
         neon_iv = vreinterpretq_f32_u32(vandq_u32(neon_mask, vreinterpretq_u32_f32(neon_iv)));
         
         neon_a = vreinterpretq_f32_u32(vandq_u32(neon_mask, vreinterpretq_u32_f32(neon_a)));
         neon_d = vreinterpretq_f32_u32(vandq_u32(neon_mask, vreinterpretq_u32_f32(neon_d)));

         // Build row
         neon_L_row[0] = neon_iu;
         neon_L_row[1] = neon_iv;
         neon_L_row[2] = vmulq_f32(neon_iu, neon_v);
         neon_L_row[3] = vmulq_f32(neon_iv, neon_ur);
         neon_L_row[4] = vsubq_f32(vmulq_f32(neon_iu, neon_ur), vmulq_f32(neon_iv, neon_v));
         neon_L_row[5] = vsubq_f32(vsubq_f32(vsubq_f32(neon_0, vmulq_f32(neon_iv, neon_v)), vmulq_f32(neon_iu, neon_ur)), vmulq_f32(neon_iv, neon_v));
         neon_L_row[6] = vmulq_f32(vsubq_f32(vsubq_f32(neon_0, vmulq_f32(neon_iv, neon_v)), vmulq_f32(neon_iu, neon_ur)), neon_ur);
         neon_L_row[7] = vmulq_f32(vsubq_f32(vsubq_f32(neon_0, vmulq_f32(neon_iv, neon_v)), vmulq_f32(neon_iu, neon_ur)), neon_v);
         neon_L_row[8] = neon_a;
         neon_L_row[9] = neon_1;

         // Fill lower triangular part 
         for (int k = 0; k < 10; k++)
         {
            for (int l = 0; l <= k; l++)
            {
               neon_LtL[k][l] = vaddq_f32(neon_LtL[k][l], vmulq_f32(neon_L_row[k], neon_L_row[l]));
            }

            neon_Lte[k] = vaddq_f32(neon_Lte[k], vmulq_f32(neon_L_row[k], neon_d));
         }

         // Increase pointers
         ptrgx += 4;
         ptrgy += 4;
         ptrd += 4;
         ptra += 4;
         ptrm += 4;

         neon_ur = vaddq_f32(neon_ur, neon_4);
      }
   }

   // Cumulate
   for (int k = 0; k < 10; k++)
   {
      for (int l = 0; l <= k; l++)
      {
         LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 0);
         LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 1);
         LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 2);
         LtL_data[k][l] += vgetq_lane_f32(neon_LtL[k][l], 3);
      }

      Lte_data[k] += vgetq_lane_f32(neon_Lte[k], 0);
      Lte_data[k] += vgetq_lane_f32(neon_Lte[k], 1);
      Lte_data[k] += vgetq_lane_f32(neon_Lte[k], 2);
      Lte_data[k] += vgetq_lane_f32(neon_Lte[k], 3);
   }

   // Symmetrise
   for (int k = 0; k < 10; k++)
   {
      for (int l = 0; l <= k; l++)
      {
         LtL_data[l][k] = LtL_data[k][l];
      }
   }

// function_terminate:
   return error;
}
