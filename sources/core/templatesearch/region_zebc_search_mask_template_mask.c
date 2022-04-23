//============================================================================
//
//    OPENROX   : File region_zebc_search_mask_template_mask.c
//
//    Contents  : Implementation of the template matching module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "region_zebc_search_mask_template_mask.h"
#include <baseproc/maths/maths_macros.h>
#include <baseproc/image/integral/integral.h>
#include <float.h>
#include <inout/system/errors_print.h>

// inlcudes for zebc
#include <baseproc/maths/linalg/matrix.h>

Rox_ErrorCode rox_array2d_float_region_zebc_search_mask_template_mask (
   Rox_Float * res_score,
   Rox_Sint * ures,
   Rox_Sint * vres,
   const Rox_Array2D_Float I,
   const Rox_Imask I_mask,
   const Rox_Array2D_Float T,
   const Rox_Imask T_mask,
   const Rox_Float zncc_threshold
)
{
   // Define the block size r
   #define  BLOCK_SIZE  5
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double cc = 0.0, zncc = 0.0;
   // A better init of zncc_max will speed up the computation
   // It would be better to set zncc_max to the threshold used
   // to accept or reject the result
   // Rox_Double zncc_max = -FLT_MAX;
   Rox_Double zncc_max = zncc_threshold; // given by user, should be tested if -1 < zncc_threshold < 1

   Rox_Uint end = 0;
   Rox_Double count_T = 0.0, sum_T = 0.0, mean_T = 0.0, norm2_T = 0.0;
   Rox_Double denom_global = 0.0, denom_local = 0.0, gamma = 0.0;

   // Per sub-region buffers
   Rox_Double count_Icts = 0.0, sum_Icts = 0.0, mean_Icts = 0.0, norm2_Icts = 0.0;
   Rox_Double * count_Tt = NULL, * count_Ict = NULL, * count_Tts = NULL;
   Rox_Double * sum_Tt = NULL, * sum_Ict = NULL, * sum_Tts = NULL;
   Rox_Double * norm2_Tt = NULL, * norm2_Ict = NULL, *norm2_Tts = NULL;
   Rox_Double * mean_Tt = NULL, *mean_Ict = NULL, *mean_Tts = NULL;
   Rox_Double * Bppt = NULL;
   Rox_Double * Bpt = NULL;
   Rox_Double * mut = NULL;
   Rox_Uint * block_height = NULL;
   Rox_Uint * block_pos = NULL;
   Rox_Uint count = 0;

   Rox_Sint u = 0, v = 0, t = 0, k = 0, l = 0;
   Rox_Uint flagStop = 0;
   Rox_Double Bpp = 0.0, Bb = 0.0;
   
   if (!res_score || !ures || !vres || !I || !I_mask || !T || !T_mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Retrieve input data pointers
   Rox_Float ** I_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &I_data, I);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** T_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &T_data, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint T_width = 0, T_height = 0;
   error = rox_array2d_float_get_size(&T_height, &T_width, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint I_width = 0, I_height;
   error = rox_array2d_float_get_size(&I_height, &I_width, I);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint C_width = I_width-T_width, C_height = I_height-T_height;

   // Declare Integral
   Rox_Matrix Is = NULL;
   error = rox_matrix_new(&Is, I_height, I_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Matrix Is2 = NULL;
   error = rox_matrix_new(&Is2, I_height, I_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Matrix Isc = NULL;
   error = rox_matrix_new(&Isc, I_height, I_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Is_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Is_data, Is);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Is2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Is2_data, Is2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Isc_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Isc_data, Isc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Matrix Ts = NULL;
   error = rox_matrix_new(&Ts, T_height, T_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Matrix Ts2 = NULL;
   error = rox_matrix_new(&Ts2, T_height, T_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Matrix Tsc = NULL;
   error = rox_matrix_new(&Tsc, T_height, T_width);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** Ts_data = NULL; 
   error = rox_array2d_double_get_data_pointer_to_pointer( &Ts_data, Ts);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ts2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Ts2_data, Ts2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Tsc_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Tsc_data, Tsc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Blocks sizes
   Rox_Sint nb_blocks = T_height / BLOCK_SIZE;
   if (T_height % BLOCK_SIZE) nb_blocks++;

   // Compute integral images
   error = rox_image_integral_sqr_double(Isc, Is, Is2, I_mask, I);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_integral_sqr_double(Tsc, Ts, Ts2, T_mask, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //rox_image_integral_sqr_double_nomask(Isc, Is, Is2, I);
   //rox_image_integral_sqr_double_nomask(Tsc, Ts, Ts2, T);

   // Get stats on template
   error = rox_getValFromIntegral(&count_T, Tsc_data, 0, 0, T_width, T_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_getValFromIntegral(&sum_T, Ts_data, 0, 0, T_width, T_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_getValFromIntegral(&norm2_T, Ts2_data, 0, 0, T_width, T_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   mean_T = sum_T / count_T;

   // Computation buffers
   Bpt = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   Bppt = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   mut = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   count_Tt = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   sum_Tt = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   norm2_Tt = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   mean_Tt = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   count_Tts = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   sum_Tts = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   norm2_Tts = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   mean_Tts = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   count_Ict = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   sum_Ict = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   norm2_Ict = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   mean_Ict = (Rox_Double *) malloc(sizeof(Rox_Double)*nb_blocks);
   block_height = (Rox_Uint *) malloc(sizeof(Rox_Uint)*nb_blocks);
   block_pos = (Rox_Uint *) malloc(sizeof(Rox_Uint)*nb_blocks);

   for (k = 0; k < (Rox_Sint) nb_blocks; k++)
   {
      if (k < (Rox_Sint) (nb_blocks - 1)) block_height[k] = BLOCK_SIZE;
      else block_height[k] = T_height - (BLOCK_SIZE * (nb_blocks - 1));
      if (k ==0) block_pos[k] = 0;
      else block_pos[k] = block_pos[k-1] + block_height[k-1];

      error = rox_getValFromIntegral(&count_Tt[k], Tsc_data, 0, block_pos[k], T_width, block_height[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_getValFromIntegral(&sum_Tt[k], Ts_data, 0, block_pos[k], T_width, block_height[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_getValFromIntegral(&norm2_Tt[k], Ts2_data, 0, block_pos[k], T_width, block_height[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      mean_Tt[k] = sum_Tt[k] / count_Tt[k];

      // Get stats for summed sub region
      end = block_pos[k] + block_height[k];
      error = rox_getValFromIntegral(&count_Tts[k], Tsc_data, 0, 0, T_width, end);
      ROX_ERROR_CHECK_TERMINATE ( error );
     
      error = rox_getValFromIntegral(&sum_Tts[k], Ts_data, 0, 0, T_width, end);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_getValFromIntegral(&norm2_Tts[k], Ts2_data, 0, 0, T_width, end);
      ROX_ERROR_CHECK_TERMINATE ( error );

      mean_Tts[k] = sum_Tt[k] / count_Tt[k];
   }

   *ures = 0;
   *vres = 0;

   // Rox_Uint elim1 = 0;
   // Rox_Uint elim2 = 0;

   // For each pixel in the search region
   for (v = 0; v < (Rox_Sint)C_height; v++)
   {
      for (u = 0; u < (Rox_Sint)C_width; u++)
      {
         // Get stats on current global region
         Rox_Double count_Ic = 0.0;
         error = rox_getValFromIntegral(&count_Ic, Isc_data, u, v, T_width, T_height);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double sum_Ic = 0.0;
         error = rox_getValFromIntegral(&sum_Ic, Is_data, u, v, T_width, T_height);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double norm2_Ic = 0.0;
         error = rox_getValFromIntegral(&norm2_Ic, Is2_data, u, v, T_width, T_height);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double mean_Ic = sum_Ic / count_Ic;
         denom_global = sqrt(norm2_Ic - sum_Ic * mean_Ic)*sqrt(norm2_T - sum_T * mean_T);

         // The inventors of ZEBC observed experimetally that it is better to compute the bound in equation (10)
         // before the bound Bb in equation (8)

         // Computing the upper bound in equation (7)
         Bpp = 0.0;
         for (t = 0; t < (Rox_Sint)nb_blocks; t++)
         {
            // Get stats on current sub-region
            error = rox_getValFromIntegral(&count_Ict[t], Isc_data, u, v + block_pos[t], T_width, block_height[t]);
            error = rox_getValFromIntegral(&sum_Ict[t], Is_data, u, v + block_pos[t], T_width, block_height[t]);
            error = rox_getValFromIntegral(&norm2_Ict[t], Is2_data, u, v + block_pos[t], T_width, block_height[t]);
            mean_Ict[t] = sum_Ict[t] / count_Ict[t];

            // Computing the upper bound in equation (7)
            Bppt[t] = sqrt(norm2_Ict[t]) * sqrt(norm2_Tt[t]) + count_Tt[t] * (mean_Ic * mean_T - mean_Tt[t] * mean_Ic - mean_T * mean_Ict[t]);
            Bpp += Bppt[t];
         }

         // Checking upper bound to eliminate, based on cauchy-schwartz inequality
         if ((Bpp / denom_global) < zncc_max)
         {
            continue;
         }

         // Computing the upper bound in equation (5)
         Bb = 0.0;
         for (t = 0; t < (Rox_Sint)nb_blocks; t++)
         {
            // Computing the upper bound in equation (5)
            Bpt[t] = sqrt(norm2_Tt[t] + count_Tt[t] * mean_T * (mean_T - 2.0 * mean_Tt[t])) * sqrt(norm2_Ict[t] + count_Ict[t] * mean_Ic * (mean_Ic - 2.0 * mean_Ict[t]));
            Bb += ROX_MIN(Bpt[t], Bppt[t]);
         }
         if ((Bb / denom_global) < zncc_max)
         {
            continue;
         }

         // Computing cross correlation
         cc = 0.0;
         flagStop = 0;

         for (t = 0; t < (Rox_Sint) nb_blocks; t++)
         {
            Rox_Sint height = block_height[t];
            Rox_Uint pos = block_pos[t];
            end = height + pos;

            mut[t] = 0.0;
            for (k = 0; k < (Rox_Sint) height; k++)
            {
               for (l = 0; l < (Rox_Sint) T_width; l++)
               {
                  // Computing cross correlation
                  mut[t] += (I_data[v+k+pos][u+l]) * (T_data[k+pos][l]);
               }
            }

            // (eq 6) Applying zero mean
            mut[t] +=  count_Tt[t] * (mean_Ic * mean_T - mean_Tt[t] * mean_Ic - mean_T * mean_Ict[t]);

            // (eq 11-12)
            if (t == 0) gamma = Bpp - ROX_MIN(Bpt[0],Bppt[0]) + mut[0];
            else gamma = gamma - ROX_MIN(Bpt[t],Bppt[t]) + mut[t];

            // Computing local norm
            error = rox_getValFromIntegral(&count_Icts, Isc_data, u, v, T_width, end);
            error = rox_getValFromIntegral(&sum_Icts, Is_data, u, v, T_width, end);
            error = rox_getValFromIntegral(&norm2_Icts, Is2_data, u, v, T_width, end);
            mean_Icts = sum_Icts / count_Icts;
            denom_local = sqrt(norm2_Icts - sum_Icts * mean_Icts)*sqrt(norm2_Tts[t] - sum_Tts[t] * mean_Tts[t]);
            if ((gamma / denom_local) < zncc_max)
            {
               flagStop = 1;
               break;
            }

            cc += mut[t];
        }

         // Candidate will never be elected, continue
         if (flagStop)
         {
            continue;
         }

         count++;
         // Potential candidate, compute zncc and update zncc_max if necessary
         zncc = cc/denom_global;
         if (zncc > zncc_max)
         {
            zncc_max = zncc;
            *res_score = (Rox_Float) (0.5 * (zncc_max + 1.0));
            *ures = u;
            *vres = v;
         }
      }
   }

function_terminate:
   // Delete memory
   rox_matrix_del(&Is);
   rox_matrix_del(&Is2);
   rox_matrix_del(&Isc);

   rox_matrix_del(&Ts);
   rox_matrix_del(&Ts2);
   rox_matrix_del(&Tsc);

   free(Bpt);
   free(Bppt);
   free(mut);
   free(count_Tt);
   free(sum_Tt);
   free(norm2_Tt);
   free(mean_Tt);
   free(count_Tts);
   free(sum_Tts);
   free(norm2_Tts);
   free(mean_Tts);
   free(count_Ict);
   free(sum_Ict);
   free(norm2_Ict);
   free(mean_Ict);
   free(block_height);
   free(block_pos);

   return error;
}
