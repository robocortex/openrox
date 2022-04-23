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
#include <baseproc/calculus/jacobians/ansi_interaction_row_texture_matse3_model3d_zi.h>
#include <stdio.h>
#include <inout/system/errors_print.h>

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

   int u_ini = 0;
   int v_ini = 0;

   for (int v = 0; v < rows; v++)
   {
      double vr = (double) (v + v_ini);

      for (int u = 0; u < cols; u++ )
      {
         double L_row[8] = { 0.0 };

         if ( Im_data[v][u] == 0 ) 
         {
            continue; 
         }
         
         double ur = (double) (u + u_ini);

         double Iu_value = (double) Iu_data[v][u];
         double Iv_value = (double) Iv_data[v][u];

         double zi  = Zi_data [v][u];
         double ziu = Ziu_data[v][u];
         double ziv = Ziv_data[v][u];
         
         double e = (double) Id_data[v][u];
         double a = (double) Ia_data[v][u];

         error = rox_ansi_interaction_row_texture_matse3_model3d_zi ( L_row, ur, vr, Iu_value, Iv_value, zi, ziu, ziv, K_data, tau_data );
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         // Interaction matrix for the light affine model
         L_row[6] =   a;
         L_row[7] = 1.0;

         // Update lower triangular part of the system
         for (int k = 0; k < 8; k++)
         {
            for (int l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L_row[k] * L_row[l] ;
            }

            Lte_data[k][0] += L_row[k] * e;
         }
      }
   }

function_terminate:
   return error;
}