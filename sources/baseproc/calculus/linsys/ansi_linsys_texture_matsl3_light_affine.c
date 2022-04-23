//==============================================================================
//
//    OPENROX   : File ansi_interaction_texture_matsl3_light_affine.c
//
//    Contents  : Implementation of ansi_interaction_texture_matsl3_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_texture_matsl3_light_affine.h"
#include <system/errors/errors.h>

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
   int error = ROX_ERROR_NONE;

   for (int i = 0; i < rows; i++)
   {
      double v = (double) i;

      for (int j = 0; j < cols; j++)
      {
         double J[10];

         if (!Im_data[i][j]) continue;

         double u = (double) j;

         double d = Id_data[i][j];
         double Iu = Iu_data[i][j];
         double Iv = Iv_data[i][j];
         double z = -Iv * v;
         double w =  Iu * u;
         double temp = z - w;

         J[0] = Iu;
         J[1] = Iv;
         J[2] = Iu * v;
         J[3] = Iv * u;
         J[4] = w + z;
         J[5] = temp + z;
         J[6] = temp * u;
         J[7] = temp * v;
         J[8] = Ia_data[i][j];
         J[9] = 1.0;

         for (int k = 0; k < 10; k++)
         {
            for (int l = 0; l <= k; l++)
            {
               LtL_data[k][l] += J[k]*J[l];
            }

            Lte_data[k] += J[k] * d;
         }
      }
   }

   for (int k = 0; k < 10; k++)
   {
      for (int l = 0; l <= k; l++)
      {
         LtL_data[l][k] = LtL_data[k][l];
      }
   }

   return error;
}