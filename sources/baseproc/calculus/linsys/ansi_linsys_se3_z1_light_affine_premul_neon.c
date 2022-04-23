//==============================================================================
//
//    OPENROX   : File ansi_linsys_se3_z1_light_affine_premul_neon.c
//
//    Contents  : Implementation of linsys_se3_z1_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_se3_z1_light_affine_premul.h"

#include <system/vectorisation/neon.h>

int rox_ansi_linsys_se3_z1_light_affine_premul (
   double ** LtL_data, 
   double *  Lte_data,
   double ** K_data,
   double ** T_data,
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

   float pxi, pyi, u0i, v0i;
   float ipx, ipy, iu0, iv0;
   float vr;
   float er11,er12,er13;
   float er21,er22,er23;
   float er31,er32,er33;
   float etx, ety, etz;
   float y, taux, tauy, tauz, zmzptauz, pyiytauy,suby, subyypyi;
   float *ptr_Iu, *ptr_Iv, *ptr_Id, *ptr_Ia; 
   unsigned int * ptr_Im;

   uint32x4_t neon_Im;
   float32x4_t neon_cols;
   float32x4_t neon_ur, neon_0, neon_1, neon_4;
   float32x4_t sseipx, neon_iu0, ssex, ssey, ssepxi, ssepyi, ssetaux;
   float32x4_t neon_Iu, neon_Iv, neon_Id, neon_Ia;
   float32x4_t ssepxixtaux, ssesubx, ssesuby, ssezmptauz, neon_Ivsuby, neon_Iusubx, ssesubyypyi;
   float32x4_t neon_L_row[6], neon_LtL[8][8], neon_Lte[8];
   Rox_Neon_Float uneon_ur;

   int cols4 = cols / 4;
   if (cols % 4) cols4++;
   
   pxi = K_data[0][0];
   pyi = K_data[1][1];
   u0i = K_data[0][2];
   v0i = K_data[1][2];

   ipx = 1.0 / pxi;
   ipy = 1.0 / pyi;
   iu0 = -u0i / pxi;
   iv0 = -v0i / pyi;

   er11 = T_data[0][0]; er12 = T_data[0][1]; er13 = T_data[0][2]; etx = T_data[0][3];
   er21 = T_data[1][0]; er22 = T_data[1][1]; er23 = T_data[1][2]; ety = T_data[1][3];
   er31 = T_data[2][0]; er32 = T_data[2][1]; er33 = T_data[2][2]; etz = T_data[2][3];

   taux = er11 * etx + ety * er21 + etz * er31;
   tauy = er12 * etx + ety * er22 + etz * er32;
   tauz = er13 * etx + er23 * ety + er33 * etz;
   zmzptauz = 1.0/(1.0+tauz);

   neon_cols = vdupq_n_f32(cols);
   sseipx = vdupq_n_f32(ipx);
   neon_iu0 = vdupq_n_f32(iu0);
   ssepxi = vdupq_n_f32(pxi);
   ssepyi = vdupq_n_f32(pyi);
   ssetaux = vdupq_n_f32(taux);
   neon_0 = vdupq_n_f32(0);
   neon_1 = vdupq_n_f32(1);
   neon_4 = vdupq_n_f32(4);
   ssezmptauz = vdupq_n_f32(zmzptauz);

   uneon_ur.tab[0] = 0;
   uneon_ur.tab[1] = 1;
   uneon_ur.tab[2] = 2;
   uneon_ur.tab[3] = 3;

   // Init to 0
   for (Rox_Sint k = 0; k < 8; k++)
   {
      for (Rox_Sint l = 0; l <= k; l++)
      {
         neon_LtL[k][l] = vdupq_n_f32(0.0);
      }

      neon_Lte[k] = vdupq_n_f32(0.0);
   }

   for (Rox_Sint i = 0; i < rows; i++)
   {
      vr = (float) (i);
      y = ipy * vr + iv0;
      pyiytauy = pyi * (y + tauy);
      suby = pyiytauy*zmzptauz;
      subyypyi = suby*y + pyi;

      ptr_Im = Im_data[i];
      ptr_Iu = Iu_data[i];
      ptr_Iv = Iv_data[i];
      ptr_Id = Id_data[i];
      ptr_Ia = Ia_data[i];

      ssesubyypyi = vdupq_n_f32(subyypyi);
      ssey = vdupq_n_f32(y);
      ssesuby = vdupq_n_f32(suby);
      neon_ur = uneon_ur.ssetype;

      for (Rox_Sint j = 0; j < cols4; j++)
      {
         neon_Im = vld1q_u32(ptr_Im);
         neon_Im = vandq_u32(neon_Im, vcltq_f32(neon_ur, neon_cols));

         neon_Iu = vld1q_f32(ptr_Iu);
         neon_Iv = vld1q_f32(ptr_Iv);
         neon_Id = vld1q_f32(ptr_Id);
         neon_Ia = vld1q_f32(ptr_Ia);

         neon_Iu = vreinterpretq_f32_u32(vandq_u32(neon_Im, vreinterpretq_u32_f32(neon_Iu)));
         neon_Iv = vreinterpretq_f32_u32(vandq_u32(neon_Im, vreinterpretq_u32_f32(neon_Iv)));
         neon_Ia = vreinterpretq_f32_u32(vandq_u32(neon_Im, vreinterpretq_u32_f32(neon_Ia)));
         neon_Id = vreinterpretq_f32_u32(vandq_u32(neon_Im, vreinterpretq_u32_f32(neon_Id)));

         ssex = vaddq_f32(vmulq_f32(sseipx, neon_ur), neon_iu0);
         ssepxixtaux = vmulq_f32(ssepxi, vaddq_f32(ssex, ssetaux));
         ssesubx = vmulq_f32(ssepxixtaux, ssezmptauz);
         neon_Ivsuby = vmulq_f32(neon_Iv, ssesuby);
         neon_Iusubx = vmulq_f32(neon_Iu, ssesubx);

         neon_L_row[0] = vmulq_f32(neon_Iu, ssepxi);
         neon_L_row[1] = vmulq_f32(neon_Iv, ssepyi);
         neon_L_row[2] = vsubq_f32(neon_0, vaddq_f32(neon_Iusubx, neon_Ivsuby));
         neon_L_row[3] = vsubq_f32(neon_0, vaddq_f32(vmulq_f32(neon_Iusubx, ssey), vmulq_f32(ssesubyypyi, neon_Iv)));
         neon_L_row[4] = vaddq_f32(vmulq_f32(neon_Iu, vaddq_f32(vmulq_f32(ssesubx, ssex), ssepxi)), vmulq_f32(neon_Ivsuby, ssex));
         neon_L_row[5] = vsubq_f32(vmulq_f32(neon_Iv, vmulq_f32(ssepyi, ssex)), vmulq_f32(neon_Iu, vmulq_f32(ssepxi, ssey)));

         for (Rox_Sint k = 0; k < 6; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               neon_LtL[k][l] = vaddq_f32(neon_LtL[k][l], vmulq_f32(neon_L_row[k], neon_L_row[l]));
            }

            neon_Lte[k] = vaddq_f32(neon_Lte[k], vmulq_f32(neon_L_row[k], neon_Id));
         }

         neon_LtL[6][0] = vaddq_f32(neon_LtL[6][0], vmulq_f32(neon_L_row[0], neon_Ia));
         neon_LtL[6][1] = vaddq_f32(neon_LtL[6][1], vmulq_f32(neon_L_row[1], neon_Ia));
         neon_LtL[6][2] = vaddq_f32(neon_LtL[6][2], vmulq_f32(neon_L_row[2], neon_Ia));
         neon_LtL[6][3] = vaddq_f32(neon_LtL[6][3], vmulq_f32(neon_L_row[3], neon_Ia));
         neon_LtL[6][4] = vaddq_f32(neon_LtL[6][4], vmulq_f32(neon_L_row[4], neon_Ia));
         neon_LtL[6][5] = vaddq_f32(neon_LtL[6][5], vmulq_f32(neon_L_row[5], neon_Ia));
         neon_LtL[6][6] = vaddq_f32(neon_LtL[6][6], vmulq_f32(neon_Ia, neon_Ia));

         neon_LtL[7][0] = vaddq_f32(neon_LtL[7][0], neon_L_row[0]);
         neon_LtL[7][1] = vaddq_f32(neon_LtL[7][1], neon_L_row[1]);
         neon_LtL[7][2] = vaddq_f32(neon_LtL[7][2], neon_L_row[2]);
         neon_LtL[7][3] = vaddq_f32(neon_LtL[7][3], neon_L_row[3]);
         neon_LtL[7][4] = vaddq_f32(neon_LtL[7][4], neon_L_row[4]);
         neon_LtL[7][5] = vaddq_f32(neon_LtL[7][5], neon_L_row[5]);
         neon_LtL[7][6] = vaddq_f32(neon_LtL[7][6], neon_Ia);
         neon_LtL[7][7] = vaddq_f32(neon_LtL[7][7], neon_1);

         neon_Lte[6] = vaddq_f32(neon_Lte[6], vmulq_f32(neon_Ia, neon_Id));
         neon_Lte[7] = vaddq_f32(neon_Lte[7], neon_Id);

         ptr_Iu+=4;
         ptr_Iv+=4;
         ptr_Id+=4;
         ptr_Ia+=4;
         ptr_Im+=4;

         neon_ur = vaddq_f32(neon_ur, neon_4);
      }
   }

   // Transfer lower triangular part of the system from NEON to CPU
   for (Rox_Sint k = 0; k < 8; k++)
   {
      for (Rox_Sint l = 0; l <= k; l++)
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

   // symmetrise
   for (int k = 0; k < 8; k++)
   {
      for (int l = k; l < 8; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

// function_terminate:
   return error;
}
