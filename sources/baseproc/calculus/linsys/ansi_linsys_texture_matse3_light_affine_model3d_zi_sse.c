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
#include <baseproc/calculus/jacobians/sse_interaction_row_texture_matse3_model3d_zi.h>

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

   __m128 sse_fu = _mm_set_ps1 ( (float) K_data[0][0] );
   __m128 sse_fv = _mm_set_ps1 ( (float) K_data[1][1] );
   __m128 sse_cu = _mm_set_ps1 ( (float) K_data[0][2] );
   __m128 sse_cv = _mm_set_ps1 ( (float) K_data[1][2] );

   __m128 sse_tau1 = _mm_set_ps1 ( (float) tau_data[0][0] );
   __m128 sse_tau2 = _mm_set_ps1 ( (float) tau_data[1][0] );
   __m128 sse_tau3 = _mm_set_ps1 ( (float) tau_data[2][0] ); 

   __m128 sse_1 = _mm_set_ps1 ( 1 );
   __m128 sse_4 = _mm_set_ps1 ( 4 );

   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   __m128 sse_cols = _mm_set_ps1 ( (float) cols );
   
   __m128 sse_LtL[8][8], sse_Lte[8];

   for (int v = 0; v < rows; v++)
   {
      float vr = (float) v;
      __m128 sse_vr = _mm_set_ps1(vr);

      unsigned int  *ptr_Im = Im_data[v];
      float * ptr_Iu = Iu_data[v];
      float * ptr_Iv = Iv_data[v];
      float * ptr_Id = Id_data[v];
      float * ptr_Ia = Ia_data[v];

      float * ptr_Zi  = Zi_data[v] ;
      float * ptr_Ziu = Ziu_data[v];
      float * ptr_Ziv = Ziv_data[v];

      __m128 sse_ur = _mm_set_ps(3, 2, 1, 0);

      // Set avx_LtL and avx_Lte to 0
      for ( int k = 0; k < 8; k++ )
      {
         for ( int l = 0; l <= k; l++ )
         {
            sse_LtL[k][l] = _mm_setzero_ps();
         }
         sse_Lte[k] = _mm_setzero_ps();
      }

      for ( int u = 0; u < cols4; u++ )
      {
         __m128 sse_Im = _mm_loadu_ps( (float*) ptr_Im);
         sse_Im = _mm_and_ps(sse_Im, _mm_cmplt_ps(sse_ur, sse_cols));
         
         // unsigned int mask = _mm_movemask_ps(sse_Im);
         
         // // rox_mm128_printf_float(sse_Im);

         // if (mask)
         // {
            __m128 sse_L_row[8];

            __m128 sse_Iu  = _mm_loadu_ps ( ptr_Iu );
            __m128 sse_Iv  = _mm_loadu_ps ( ptr_Iv );
            __m128 sse_Id  = _mm_loadu_ps ( ptr_Id );
            __m128 sse_Ia  = _mm_loadu_ps ( ptr_Ia );
            __m128 sse_Zi  = _mm_loadu_ps ( ptr_Zi );
            __m128 sse_Ziu = _mm_loadu_ps ( ptr_Ziu);
            __m128 sse_Ziv = _mm_loadu_ps ( ptr_Ziv);

            sse_Iu  = _mm_and_ps(sse_Iu, sse_Im);
            sse_Iv  = _mm_and_ps(sse_Iv, sse_Im);

            sse_Id  = _mm_and_ps(sse_Id, sse_Im);
            sse_Ia  = _mm_and_ps(sse_Ia, sse_Im);
            
            sse_Zi  = _mm_and_ps(sse_Zi , sse_Im);
            sse_Ziu = _mm_and_ps(sse_Ziu, sse_Im);
            sse_Ziv = _mm_and_ps(sse_Ziv, sse_Im);

            error = rox_sse_interaction_row_texture_matse3_model3d_zi ( sse_L_row, sse_ur, sse_vr, sse_Iu, sse_Iv, sse_Zi, sse_Ziu, sse_Ziv, sse_fu, sse_fv, sse_cu, sse_cv, sse_tau1, sse_tau2, sse_tau3 );
            if (error) { goto function_terminate; }

            // Interaction matrix for the light affine model
            sse_L_row[6] = sse_Ia;
            sse_L_row[7] = _mm_and_ps(sse_1, sse_Im);

            // Update lower triangular part of the system
            for ( int k = 0; k < 8; k++ )
            {
               for ( int l = 0; l <= k; l++ )
               {
                  sse_LtL[k][l] = _mm_add_ps(sse_LtL[k][l], _mm_mul_ps(sse_L_row[k], sse_L_row[l]));
               }
               sse_Lte[k] = _mm_add_ps(sse_Lte[k], _mm_mul_ps(sse_L_row[k], sse_Id));
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

         sse_ur = _mm_add_ps ( sse_ur, sse_4 );
      }

      // Transfer lower triangular part of the system from SSE to CPU
      for ( int k = 0; k < 8; k++ )
      {
         for ( int l = 0; l <= k; l++ )
         {
            LtL_data[k][l] += rox_mm128_hsum_ps ( sse_LtL[k][l] );
         }
         Lte_data[k][0] += rox_mm128_hsum_ps ( sse_Lte[k] );
      }
   }

function_terminate:
   return error;
}
