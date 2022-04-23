//==============================================================================
//
//    OPENROX   : File ansi_linsys_se3_light_affine_premul_sse.c
//
//    Contents  : Implementation of linsys_se3_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_se3_light_affine_premul.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/symmetrise/symmetrise.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

int rox_ansi_linsys_se3_light_affine_premul (
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

   Rox_Array2D_Float buf = NULL;

   error = rox_array2d_float_new ( &buf, 1, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   float * ptr = NULL;
   error = rox_array2d_float_get_data_pointer( &ptr, buf );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols4 = cols / 4;
   if (cols % 4) cols4++;

   float fu = (float) K_data[0][0];
   float fv = (float) K_data[1][1];
   float cu = (float) K_data[0][2];
   float cv = (float) K_data[1][2];

   float ipx = 1.0f / fu;
   float iu0 =  -cu / fu;

   float r11 = (float) T_data[0][0]; float r12 = (float) T_data[0][1]; float r13 = (float) T_data[0][2]; float tx = (float) T_data[0][3];
   float r21 = (float) T_data[1][0]; float r22 = (float) T_data[1][1]; float r23 = (float) T_data[1][2]; float ty = (float) T_data[1][3];
   float r31 = (float) T_data[2][0]; float r32 = (float) T_data[2][1]; float r33 = (float) T_data[2][2]; float tz = (float) T_data[2][3];

   float taux = -(r11 * tx + ty * r21 + tz * r31);
   float tauy = -(r12 * tx + ty * r22 + tz * r32);
   float tauz = -(r13 * tx + ty * r23 + tz * r33);
   float inv_tauz = (float) (1.0/tauz);
   
   __m128 sse_L[6], sse_LtL[8][8], sse_Lte[8];

   __m128 ssecols = _mm_set_ps1((float) cols);
   __m128 sseipx = _mm_set_ps1(ipx);
   __m128 sseiu0 = _mm_set_ps1(iu0);
   __m128 sse_fu = _mm_set_ps1(fu);
   __m128 sse_fv = _mm_set_ps1(fv);

   __m128 sse_taux = _mm_set_ps1(taux);
   __m128 sse_tauy = _mm_set_ps1(tauy);
   __m128 sse_inv_tauz = _mm_set_ps1(inv_tauz);

   __m128 sse_0 = _mm_set_ps1(0.0f);
   __m128 sse_1 = _mm_set_ps1(1.0f);
   __m128 sse_4 = _mm_set_ps1(4.0f);

   for ( int i = 0; i < rows; i++)
   {
      float vr = (float)(i);
      float y = (vr - cv)/fv;

      unsigned int * ptr_m = Im_data[i];
      float * ptr_Iu = Iu_data[i];
      float * ptr_Iv = Iv_data[i];
      float * ptr_d = Id_data[i];
      float * ptr_a = Ia_data[i];

      __m128 sse_y = _mm_set_ps1(y);
      __m128 sse_dy = _mm_sub_ps(sse_y, sse_tauy);

      __m128 sse_ur = _mm_set_ps(3, 2, 1, 0);

      for (int k = 0; k < 8; k++)
      {
         for (int l = 0; l <= k; l++)
         {
            sse_LtL[k][l] = _mm_setzero_ps();
         }

         sse_Lte[k] = _mm_setzero_ps();
      }

      for (int j = 0; j < cols4; j++)
      {
         __m128 sse_mask = _mm_load_ps ( (float*) ptr_m );
         sse_mask = _mm_and_ps ( sse_mask, _mm_cmplt_ps(sse_ur, ssecols) );

         unsigned int gmask = _mm_movemask_ps(sse_mask);

         if (gmask)
         {
            __m128 sse_Iu = _mm_load_ps(ptr_Iu);
            __m128 sse_Iv = _mm_load_ps(ptr_Iv);
            __m128 ssed = _mm_load_ps(ptr_d);
            __m128 ssea = _mm_load_ps(ptr_a);

            sse_Iu = _mm_and_ps(sse_Iu, sse_mask);
            sse_Iv = _mm_and_ps(sse_Iv, sse_mask);
            ssea = _mm_and_ps(ssea, sse_mask);
            ssed = _mm_and_ps(ssed, sse_mask);

            // ipx * ur + iu0
            __m128 sse_x = _mm_add_ps(_mm_mul_ps(sseipx, sse_ur), sseiu0);
            __m128 sse_dx = _mm_sub_ps(sse_x, sse_taux);

            // gu = Iu * fu
            __m128 sse_gu = _mm_mul_ps(sse_Iu, sse_fu);
            // gv = Iv * fv
            __m128 sse_gv = _mm_mul_ps(sse_Iv, sse_fv);

            // sse_gudx = gu * (x - taux) = Iu * fu * (x - taux);
            __m128 sse_gudx = _mm_mul_ps(sse_gu, sse_dx);
            // sse_gvdy = gv * (y - tauy) = Iv * fv * (y - tauy);
            __m128 sse_gvdy = _mm_mul_ps(sse_gv, sse_dy);

            // gw = (gu * (x - taux)  + gv * (y - tauy))/tauz;
            __m128 sse_gw = _mm_mul_ps(_mm_add_ps(sse_gudx, sse_gvdy), sse_inv_tauz);

            sse_L[0] = sse_gu;

            sse_L[1] = sse_gv;

            sse_L[2] = sse_gw;

            //   gw * y
            sse_L[3] = _mm_mul_ps(sse_gw, sse_y);
            // - gw * x
            sse_L[4] = _mm_mul_ps(_mm_sub_ps(sse_0, sse_gw), sse_x);
            // Iv * ssefv * ssex - Iv * ssefu * ssey = Iv * fv * x - Iu * fu * y
            sse_L[5] = _mm_sub_ps(_mm_mul_ps(sse_gv, sse_x), _mm_mul_ps(sse_gu, sse_y));

            for (int k = 0; k < 6; k++)
            {
               for (int l = 0; l <= k; l++)
               {
                  sse_LtL[k][l] = _mm_add_ps(sse_LtL[k][l], _mm_mul_ps(sse_L[k], sse_L[l]));
               }

               sse_Lte[k] = _mm_add_ps(sse_Lte[k], _mm_mul_ps(sse_L[k], ssed));
            }

            sse_LtL[6][0] = _mm_add_ps(sse_LtL[6][0], _mm_mul_ps(sse_L[0], ssea));
            sse_LtL[6][1] = _mm_add_ps(sse_LtL[6][1], _mm_mul_ps(sse_L[1], ssea));
            sse_LtL[6][2] = _mm_add_ps(sse_LtL[6][2], _mm_mul_ps(sse_L[2], ssea));
            sse_LtL[6][3] = _mm_add_ps(sse_LtL[6][3], _mm_mul_ps(sse_L[3], ssea));
            sse_LtL[6][4] = _mm_add_ps(sse_LtL[6][4], _mm_mul_ps(sse_L[4], ssea));
            sse_LtL[6][5] = _mm_add_ps(sse_LtL[6][5], _mm_mul_ps(sse_L[5], ssea));
            sse_LtL[6][6] = _mm_add_ps(sse_LtL[6][6], _mm_mul_ps(ssea, ssea));

            sse_LtL[7][0] = _mm_add_ps(sse_LtL[7][0], sse_L[0]);
            sse_LtL[7][1] = _mm_add_ps(sse_LtL[7][1], sse_L[1]);
            sse_LtL[7][2] = _mm_add_ps(sse_LtL[7][2], sse_L[2]);
            sse_LtL[7][3] = _mm_add_ps(sse_LtL[7][3], sse_L[3]);
            sse_LtL[7][4] = _mm_add_ps(sse_LtL[7][4], sse_L[4]);
            sse_LtL[7][5] = _mm_add_ps(sse_LtL[7][5], sse_L[5]);
            sse_LtL[7][6] = _mm_add_ps(sse_LtL[7][6], ssea);
            sse_LtL[7][7] = _mm_add_ps(sse_LtL[7][7], sse_1);

            sse_Lte[6] = _mm_add_ps(sse_Lte[6], _mm_mul_ps(ssea, ssed));
            sse_Lte[7] = _mm_add_ps(sse_Lte[7], ssed);
         }

         ptr_Iu+=4;
         ptr_Iv+=4;
         ptr_d+=4;
         ptr_a+=4;
         ptr_m+=4;

         sse_ur = _mm_add_ps(sse_ur, sse_4);
      }

      for (int k = 0; k < 8; k++)
      {
         for (int l = 0; l <= k; l++)
         {
            sse_LtL[k][l] = _mm_hadd_ps(sse_LtL[k][l], sse_LtL[k][l]);
            sse_LtL[k][l] = _mm_hadd_ps(sse_LtL[k][l], sse_LtL[k][l]);
            _mm_store1_ps(ptr, sse_LtL[k][l]);
            LtL_data[k][l] += ptr[0];
         }

         sse_Lte[k] = _mm_hadd_ps(sse_Lte[k], sse_Lte[k]);
         sse_Lte[k] = _mm_hadd_ps(sse_Lte[k], sse_Lte[k]);
         _mm_store1_ps(ptr, sse_Lte[k]);
         Lte_data[k] += ptr[0];
      }
   }

   // Symmetrise
   for (int k = 0; k < 8; k++)
   {
      for (int l = k; l < 8; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   rox_array2d_float_del(&buf);

   return error;
}

int rox_2ansi_linsys_se3_light_affine_premul (
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

   Rox_Array2D_Float buf = NULL;

   error = rox_array2d_float_new ( &buf, 1, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   float * ptr = NULL;
   error = rox_array2d_float_get_data_pointer( &ptr, buf );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols4 = cols / 4;
   if (cols % 4) cols4++;

   float fu = (float) K_data[0][0];
   float fv = (float) K_data[1][1];
   float cu = (float) K_data[0][2];
   float cv = (float) K_data[1][2];

   float ipx = 1.0f / fu;
   float iu0 =  -cu / fu;

   float r11 = (float) T_data[0][0]; float r12 = (float) T_data[0][1]; float r13 = (float) T_data[0][2]; float tx = (float) T_data[0][3];
   float r21 = (float) T_data[1][0]; float r22 = (float) T_data[1][1]; float r23 = (float) T_data[1][2]; float ty = (float) T_data[1][3];
   float r31 = (float) T_data[2][0]; float r32 = (float) T_data[2][1]; float r33 = (float) T_data[2][2]; float tz = (float) T_data[2][3];

   float taux = -(r11 * tx + ty * r21 + tz * r31);
   float tauy = -(r12 * tx + ty * r22 + tz * r32);
   float tauz = -(r13 * tx + ty * r23 + tz * r33);
   float inv_tauz = (float) (1.0/tauz);
   
   __m128 sse_L[6], sse_LtL[8][8], sse_Lte[8];

   __m128 ssecols = _mm_set_ps1((float) cols);
   __m128 sseipx = _mm_set_ps1(ipx);
   __m128 sseiu0 = _mm_set_ps1(iu0);
   __m128 sse_fu = _mm_set_ps1(fu);
   __m128 sse_fv = _mm_set_ps1(fv);

   __m128 sse_taux = _mm_set_ps1(taux);
   __m128 sse_tauy = _mm_set_ps1(tauy);
   __m128 sse_inv_tauz = _mm_set_ps1(inv_tauz);

   __m128 sse_0 = _mm_set_ps1(0.0f);
   __m128 sse_1 = _mm_set_ps1(1.0f);
   __m128 sse_4 = _mm_set_ps1(4.0f);

   for ( int i = 0; i < rows; i++)
   {
      float vr = (float)(i);
      float y = (vr - cv)/fv;

      unsigned int * ptr_m = Im_data[i];
      float * ptr_Iu = Iu_data[i];
      float * ptr_Iv = Iv_data[i];
      float * ptr_d = Id_data[i];
      float * ptr_a = Ia_data[i];

      __m128 sse_y = _mm_set_ps1(y);
      __m128 sse_dy = _mm_sub_ps(sse_y, sse_tauy);

      __m128 sse_ur = _mm_set_ps(3, 2, 1, 0);

      for (int k = 0; k < 8; k++)
      {
         for (int l = 0; l <= k; l++)
         {
            sse_LtL[k][l] = _mm_setzero_ps();
         }

         sse_Lte[k] = _mm_setzero_ps();
      }

      for (int j = 0; j < cols4; j++)
      {
         __m128 sse_mask = _mm_load_ps ( (float*) ptr_m );
         sse_mask = _mm_and_ps ( sse_mask, _mm_cmplt_ps(sse_ur, ssecols) );

         unsigned int gmask = _mm_movemask_ps(sse_mask);

         if (gmask)
         {
            __m128 sse_Iu = _mm_load_ps(ptr_Iu);
            __m128 sse_Iv = _mm_load_ps(ptr_Iv);
            __m128 ssed = _mm_load_ps(ptr_d);
            __m128 ssea = _mm_load_ps(ptr_a);

            sse_Iu = _mm_and_ps(sse_Iu, sse_mask);
            sse_Iv = _mm_and_ps(sse_Iv, sse_mask);
            ssea = _mm_and_ps(ssea, sse_mask);
            ssed = _mm_and_ps(ssed, sse_mask);

            // ipx * ur + iu0
            __m128 sse_x = _mm_add_ps(_mm_mul_ps(sseipx, sse_ur), sseiu0);
            __m128 sse_dx = _mm_sub_ps(sse_x, sse_taux);

            // gu = Iu * fu
            __m128 sse_gu = _mm_mul_ps(sse_Iu, sse_fu);
            // gv = Iv * fv
            __m128 sse_gv = _mm_mul_ps(sse_Iv, sse_fv);

            // sse_gudx = gu * (x - taux) = Iu * fu * (x - taux);
            __m128 sse_gudx = _mm_mul_ps(sse_gu, sse_dx);
            // sse_gvdy = gv * (y - tauy) = Iv * fv * (y - tauy);
            __m128 sse_gvdy = _mm_mul_ps(sse_gv, sse_dy);

            // gw = (gu * (x - taux)  + gv * (y - tauy))/tauz;
            __m128 sse_gw = _mm_mul_ps(_mm_add_ps(sse_gudx, sse_gvdy), sse_inv_tauz);

            sse_L[0] = sse_gu;

            sse_L[1] = sse_gv;

            sse_L[2] = sse_gw;

            //   gw * y
            sse_L[3] = _mm_mul_ps(sse_gw, sse_y);
            // - gw * x
            sse_L[4] = _mm_mul_ps(_mm_sub_ps(sse_0, sse_gw), sse_x);
            // Iv * ssefv * ssex - Iv * ssefu * ssey = Iv * fv * x - Iu * fu * y
            sse_L[5] = _mm_sub_ps(_mm_mul_ps(sse_gv, sse_x), _mm_mul_ps(sse_gu, sse_y));

            for (int k = 0; k < 6; k++)
            {
               for (int l = 0; l <= k; l++)
               {
                  sse_LtL[k][l] = _mm_add_ps(sse_LtL[k][l], _mm_mul_ps(sse_L[k], sse_L[l]));
               }

               sse_Lte[k] = _mm_add_ps(sse_Lte[k], _mm_mul_ps(sse_L[k], ssed));
            }

            sse_LtL[6][0] = _mm_add_ps(sse_LtL[6][0], _mm_mul_ps(sse_L[0], ssea));
            sse_LtL[6][1] = _mm_add_ps(sse_LtL[6][1], _mm_mul_ps(sse_L[1], ssea));
            sse_LtL[6][2] = _mm_add_ps(sse_LtL[6][2], _mm_mul_ps(sse_L[2], ssea));
            sse_LtL[6][3] = _mm_add_ps(sse_LtL[6][3], _mm_mul_ps(sse_L[3], ssea));
            sse_LtL[6][4] = _mm_add_ps(sse_LtL[6][4], _mm_mul_ps(sse_L[4], ssea));
            sse_LtL[6][5] = _mm_add_ps(sse_LtL[6][5], _mm_mul_ps(sse_L[5], ssea));
            sse_LtL[6][6] = _mm_add_ps(sse_LtL[6][6], _mm_mul_ps(ssea, ssea));

            sse_LtL[7][0] = _mm_add_ps(sse_LtL[7][0], sse_L[0]);
            sse_LtL[7][1] = _mm_add_ps(sse_LtL[7][1], sse_L[1]);
            sse_LtL[7][2] = _mm_add_ps(sse_LtL[7][2], sse_L[2]);
            sse_LtL[7][3] = _mm_add_ps(sse_LtL[7][3], sse_L[3]);
            sse_LtL[7][4] = _mm_add_ps(sse_LtL[7][4], sse_L[4]);
            sse_LtL[7][5] = _mm_add_ps(sse_LtL[7][5], sse_L[5]);
            sse_LtL[7][6] = _mm_add_ps(sse_LtL[7][6], ssea);
            sse_LtL[7][7] = _mm_add_ps(sse_LtL[7][7], sse_1);

            sse_Lte[6] = _mm_add_ps(sse_Lte[6], _mm_mul_ps(ssea, ssed));
            sse_Lte[7] = _mm_add_ps(sse_Lte[7], ssed);
         }

         ptr_Iu+=4;
         ptr_Iv+=4;
         ptr_d+=4;
         ptr_a+=4;
         ptr_m+=4;

         sse_ur = _mm_add_ps(sse_ur, sse_4);
      }

      for (int k = 0; k < 8; k++)
      {
         for (int l = 0; l <= k; l++)
         {
            sse_LtL[k][l] = _mm_hadd_ps(sse_LtL[k][l], sse_LtL[k][l]);
            sse_LtL[k][l] = _mm_hadd_ps(sse_LtL[k][l], sse_LtL[k][l]);
            _mm_store1_ps(ptr, sse_LtL[k][l]);
            LtL_data[k][l] += ptr[0];
         }

         sse_Lte[k] = _mm_hadd_ps(sse_Lte[k], sse_Lte[k]);
         sse_Lte[k] = _mm_hadd_ps(sse_Lte[k], sse_Lte[k]);
         _mm_store1_ps(ptr, sse_Lte[k]);
         Lte_data[k] += ptr[0];
      }
   }

   // Symmetrise
   for (int k = 0; k < 8; k++)
   {
      for (int l = k; l < 8; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   rox_array2d_float_del(&buf);

   return error;
}
