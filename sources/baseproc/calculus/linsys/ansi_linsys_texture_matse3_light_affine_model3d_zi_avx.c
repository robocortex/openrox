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
#include <baseproc/calculus/jacobians/avx_interaction_row_texture_matse3_model3d_zi.h>

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

   __m256 avx_fu = _mm256_set1_ps ( (float) K_data[0][0] );
   __m256 avx_fv = _mm256_set1_ps ( (float) K_data[1][1] );
   __m256 avx_cu = _mm256_set1_ps ( (float) K_data[0][2] );
   __m256 avx_cv = _mm256_set1_ps ( (float) K_data[1][2] );

   __m256 avx_tau1 = _mm256_set1_ps ( (float) tau_data[0][0] );
   __m256 avx_tau2 = _mm256_set1_ps ( (float) tau_data[1][0] );
   __m256 avx_tau3 = _mm256_set1_ps ( (float) tau_data[2][0] );

   __m256 avx_1 = _mm256_set1_ps(1);
   __m256 avx_8 = _mm256_set1_ps(8);

   int cols8 = cols / 8;
   if (cols % 8) cols8++;
   __m256 avx_cols = _mm256_set1_ps( (float) cols);

   __m256 avx_LtL[8][8], avx_Lte[8];

   for (int v = 0; v < rows; v++)
   {
      float vr = (float) (v);
      __m256 avx_vr = _mm256_set1_ps ( vr );

      Rox_Uint  * ptr_Im = Im_data[v];

      Rox_Float * ptr_Iu = Iu_data[v];
      Rox_Float * ptr_Iv = Iv_data[v];
      
      Rox_Float * ptr_Id = Id_data[v];
      Rox_Float * ptr_Ia = Ia_data[v];

      Rox_Float * ptr_Zi = Zi_data[v];
      Rox_Float * ptr_Ziu = Ziu_data[v];
      Rox_Float * ptr_Ziv = Ziv_data[v];

      __m256 avx_ur = _mm256_set_ps ( 7, 6, 5, 4, 3, 2, 1, 0 );

      // Set avx_LtL and avx_Lte to 0
      for ( int k = 0; k < 8; k++ )
      {
         for ( int l = 0; l <= k; l++ )
         {
            avx_LtL[k][l] = _mm256_setzero_ps();
         }
         avx_Lte[k] = _mm256_setzero_ps();
      }

      for ( int u = 0; u < cols8; u++ )
      {  
         #ifdef LOAD_INT
         __m256i avxi_Im = _mm256_loadu_si256 ( (__m256i *) ptr_Im );
         __m256 avx_Im = _mm256_castsi256_ps ( avxi_Im );
         #else
            // Could be changed to _mm256_load_ps if we force 32 bits memory allocaltion alignment
            __m256 avx_Im = _mm256_loadu_ps ( (float *) ptr_Im );
         #endif

         avx_Im = _mm256_and_ps ( avx_Im, _mm256_cmp_ps ( avx_ur, avx_cols, _CMP_LT_OS ) );

         // Rox_Uint mask = _mm256_movemask_ps(avx_Im);
         // rox_mm256_printf_float(avx_Im);

         // if (mask)
         // {
            __m256 avx_L_row[8];

            __m256 avx_Iu  = _mm256_loadu_ps(ptr_Iu);
            __m256 avx_Iv  = _mm256_loadu_ps(ptr_Iv);
            __m256 avx_Id  = _mm256_loadu_ps(ptr_Id);
            __m256 avx_Ia  = _mm256_loadu_ps(ptr_Ia);
            __m256 avx_Zi  = _mm256_loadu_ps(ptr_Zi );
            __m256 avx_Ziu = _mm256_loadu_ps(ptr_Ziu);
            __m256 avx_Ziv = _mm256_loadu_ps(ptr_Ziv);

            // Could be changed to _mm256_load_ps if we force 32 bits memory allocaltion alignment
            avx_Iu  = _mm256_and_ps ( avx_Iu, avx_Im );
            avx_Iv  = _mm256_and_ps ( avx_Iv, avx_Im );

            avx_Id  = _mm256_and_ps ( avx_Id, avx_Im );
            avx_Ia  = _mm256_and_ps ( avx_Ia, avx_Im );

            avx_Zi  = _mm256_and_ps ( avx_Zi, avx_Im );
            avx_Ziu = _mm256_and_ps ( avx_Ziu, avx_Im );
            avx_Ziv = _mm256_and_ps ( avx_Ziv, avx_Im );
            
            error = rox_avx_interaction_row_texture_matse3_model3d_zi ( avx_L_row, avx_ur, avx_vr, avx_Iu, avx_Iv, avx_Zi, avx_Ziu, avx_Ziv, avx_fu, avx_fv, avx_cu, avx_cv, avx_tau1, avx_tau2, avx_tau3 );
            if (error) { goto function_terminate; }

            // Interaction matrix for the light affine model
            avx_L_row[6] = avx_Ia;
            avx_L_row[7] = _mm256_and_ps ( avx_1, avx_Im );

            // Update lower triangular part of the system
            for (Rox_Sint k = 0; k < 8; k++)
            {
               for (Rox_Sint l = 0; l <= k; l++)
               {
                  avx_LtL[k][l] = _mm256_add_ps(avx_LtL[k][l], _mm256_mul_ps(avx_L_row[k], avx_L_row[l]));
               }
               avx_Lte[k] = _mm256_add_ps(avx_Lte[k], _mm256_mul_ps(avx_L_row[k], avx_Id));
            }
         // }
         
         ptr_Iu += 8;
         ptr_Iv += 8;
         ptr_Id += 8;
         ptr_Ia += 8;
         ptr_Im += 8;

         ptr_Zi += 8;
         ptr_Ziu += 8;
         ptr_Ziv += 8;

         avx_ur = _mm256_add_ps ( avx_ur, avx_8 );
      }

      // Transfer lower triangular part of the system from SSE to CPU
      for ( Rox_Sint k = 0; k < 8; k++ )
      {
         for ( Rox_Sint l = 0; l <= k; l++ )
         {
            LtL_data[k][l] += rox_mm256_hsum_ps(avx_LtL[k][l]);

         }
         Lte_data[k][0] += rox_mm256_hsum_ps(avx_Lte[k]);
      }
   }

function_terminate:
   return error;
}
