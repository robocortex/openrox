//==============================================================================
//
//    OPENROX   : File ansi_linsys_se3z1_light_affine_premul.c
//
//    Contents  : Implementation of linsys_se3z1_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_se3_z1_light_affine_premul.h"
#include <system/errors/errors.h>

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
   int error = ROX_ERROR_NONE;
   
   float fu = K_data[0][0];
   float fv = K_data[1][1];
   float cu = K_data[0][2];
   float cv = K_data[1][2];

   float er11 = T_data[0][0]; float er12 = T_data[0][1]; float er13 = T_data[0][2]; float etx = T_data[0][3];
   float er21 = T_data[1][0]; float er22 = T_data[1][1]; float er23 = T_data[1][2]; float ety = T_data[1][3];
   float er31 = T_data[2][0]; float er32 = T_data[2][1]; float er33 = T_data[2][2]; float etz = T_data[2][3];

   float taux = -(er11 * etx + er21 * ety + er31 * etz);
   float tauy = -(er12 * etx + er22 * ety + er32 * etz);
   float tauz = -(er13 * etx + er23 * ety + er33 * etz);
   float tauzp1 = tauz-1.0;
   float invtauzp1 = 1.0 / tauzp1;

   for ( int i = 0; i < rows; i++)
   {
      float L[6];

      float vr = (float)(i);
      float y = (vr - cv)/fv;

      for ( int j = 0; j < cols; j++)
      {
         if (!Im_data[i][j]) continue;

         float ur = (float)(j);
         float x = (ur - cu)/fu;

         // Retrieve per pixel params
         float Iu_val = Iu_data[i][j];
         float Iv_val = Iv_data[i][j];
         float d = Id_data[i][j];
         float a = Ia_data[i][j];

         float t1 = Iv_val * fv;
         float t2 = Iu_val * fu;
         float t3 = Iu_val * fu * (x - taux) + fv * Iv_val * (y - tauy);

         // Jacobian row
         // Translation
         L[0] = t2;
         L[1] = t1;
         L[2] = t3 * invtauzp1;
         //L[3] =    (t1 - Iv_val * fvtauz + t3 * y) * invtauzp1;
         //L[4] = -  (t2 - Iu_val * futauz + t3 * x) * invtauzp1;
         // Rotation
         L[3] = - t1 + t3 * y * invtauzp1;
         L[4] =   t2 - t3 * x * invtauzp1;
         L[5] = x * t1 - y * t2;

         // Update system
         for ( int k = 0; k < 6; k++ )
         {
            for ( int l = 0; l <= k; l++ )
            {
               LtL_data[k][l] += L[k] * L[l];
            }

            Lte_data[k] += L[k] * d;
         }

         LtL_data[6][0] += L[0] * a;
         LtL_data[6][1] += L[1] * a;
         LtL_data[6][2] += L[2] * a;
         LtL_data[6][3] += L[3] * a;
         LtL_data[6][4] += L[4] * a;
         LtL_data[6][5] += L[5] * a;
         LtL_data[6][6] += a * a;

         LtL_data[7][0] += L[0];
         LtL_data[7][1] += L[1];
         LtL_data[7][2] += L[2];
         LtL_data[7][3] += L[3];
         LtL_data[7][4] += L[4];
         LtL_data[7][5] += L[5];
         LtL_data[7][6] += a;
         LtL_data[7][7] += 1;

         Lte_data[6] += a * d;
         Lte_data[7] += d;
      }
   }

   // Symmetrise
   for ( int k = 0; k < 8; k++ )
   {
      for ( int l = k; l < 8; l++ )
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

// function_terminate:
   return error;
}