//==============================================================================
//
//    OPENROX   : File ansi_linsys_se3z1_light_affine_premul_sse.c
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

#include <system/vectorisation/sse.h>

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

   float pxi = (float) K_data[0][0];
   float pyi = (float) K_data[1][1];
   float u0i = (float) K_data[0][2];
   float v0i = (float) K_data[1][2];

   float ipx =  1.0f / pxi;
   float ipy =  1.0f / pyi;
   float iu0 = -u0i / pxi;
   float iv0 = -v0i / pyi;

   float er11 = (float) T_data[0][0]; float er12 = (float) T_data[0][1]; float er13 = (float) T_data[0][2]; float etx = (float) T_data[0][3];
   float er21 = (float) T_data[1][0]; float er22 = (float) T_data[1][1]; float er23 = (float) T_data[1][2]; float ety = (float) T_data[1][3];
   float er31 = (float) T_data[2][0]; float er32 = (float) T_data[2][1]; float er33 = (float) T_data[2][2]; float etz = (float) T_data[2][3];

   float taux = er11 * etx + ety * er21 + etz * er31;
   float tauy = er12 * etx + ety * er22 + etz * er32;
   float tauz = er13 * etx + er23 * ety + er33 * etz;

   float zmzptauz = (float) (1.0/(1.0+tauz));

   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   __m128 sse_L[8], sse_LtL[8][8], sse_Lte[8];

   __m128 sse_cols = _mm_set_ps1((float) cols);
   __m128 sseipx = _mm_set_ps1(ipx);
   __m128 sse_Iu0 = _mm_set_ps1(iu0);
   __m128 ssepxi = _mm_set_ps1(pxi);
   __m128 ssepyi = _mm_set_ps1(pyi);

   __m128 ssetaux = _mm_set_ps1(taux);
   __m128 ssezmptauz = _mm_set_ps1(zmzptauz);

   __m128 sse_0 = _mm_set_ps1(0);
   __m128 sse_1 = _mm_set_ps1(1);
   __m128 sse_4 = _mm_set_ps1(4);

   for (int v = 0; v < rows; v++)
   {
      float vr = (float) (v);

      unsigned int * ptr_Im = Im_data[v];
      float * ptr_Iu = Iu_data[v];
      float * ptr_Iv = Iv_data[v];
      float * ptr_Id = Id_data[v];
      float * ptr_Ia = Ia_data[v];

      // Compute L_row
      float y = ipy * vr + iv0;
      float pyiytauy = pyi * (y + tauy);
      float suby = pyiytauy*zmzptauz;
      float subyypyi = suby*y + pyi;

      __m128 ssesubyypyi = _mm_set_ps1(subyypyi);
      __m128 ssey = _mm_set_ps1(y);
      // ssesuby = suby
      __m128 ssesuby = _mm_set_ps1(suby);

      __m128 sse_ur = _mm_set_ps(3, 2, 1, 0);

      for (int k = 0; k < 8; k++)
      {
         for (int l = 0; l <= k; l++)
         {
            sse_LtL[k][l] = _mm_setzero_ps();
         }
         sse_Lte[k] = _mm_setzero_ps();
      }

      for ( int u = 0; u < cols4; u++ )
      {
         __m128 sse_Im = _mm_load_ps((float*)ptr_Im);
         sse_Im = _mm_and_ps(sse_Im, _mm_cmplt_ps(sse_ur, sse_cols));
         Rox_Uint gmask = _mm_movemask_ps(sse_Im);

         if (gmask)
         {
            __m128 sse_Iu = _mm_load_ps(ptr_Iu);
            __m128 sse_Iv = _mm_load_ps(ptr_Iv);
            __m128 sse_Id = _mm_load_ps(ptr_Id);
            __m128 sse_Ia = _mm_load_ps(ptr_Ia);

            sse_Iu = _mm_and_ps(sse_Iu, sse_Im);
            sse_Iv = _mm_and_ps(sse_Iv, sse_Im);
            sse_Ia = _mm_and_ps(sse_Ia, sse_Im);
            sse_Id = _mm_and_ps(sse_Id, sse_Im);

            // ipx * ur + iu0
            __m128 ssex = _mm_add_ps(_mm_mul_ps(sseipx, sse_ur), sse_Iu0);
            __m128 ssepxixtaux = _mm_mul_ps(ssepxi, _mm_add_ps(ssex, ssetaux));
            __m128 ssesubx = _mm_mul_ps(ssepxixtaux, ssezmptauz);

            // sse_Ivsuby = Iv * suby = Iv * (y - tauy);
            __m128 sse_Ivsuby = _mm_mul_ps(sse_Iv, ssesuby);
            __m128 sse_Iusubx = _mm_mul_ps(sse_Iu, ssesubx);

            // Iu * pxi = Iu * fu
            sse_L[0] = _mm_mul_ps(sse_Iu, ssepxi);

            // Iv * pyi = Iv * fv
            sse_L[1] = _mm_mul_ps(sse_Iv, ssepyi);

            // t3 = Iu * fu * (x - taux)  + fv * Iv * (y - tauy);
            // 0 - ( sse_Iusubx + sse_Ivsuby )
            sse_L[2] = _mm_sub_ps(sse_0, _mm_add_ps(sse_Iusubx, sse_Ivsuby));

            //
            sse_L[3] = _mm_sub_ps(sse_0, _mm_add_ps(_mm_mul_ps(sse_Iusubx, ssey), _mm_mul_ps(ssesubyypyi, sse_Iv)));

            //
            sse_L[4] = _mm_add_ps(_mm_mul_ps(sse_Iu, _mm_add_ps(_mm_mul_ps(ssesubx, ssex), ssepxi)), _mm_mul_ps(sse_Ivsuby, ssex));

            // Iv * ssepyi * ssex - Iv * ssepxi * ssey = Iv * fv * x - Iu * fu * y
            sse_L[5] = _mm_sub_ps(_mm_mul_ps(sse_Iv, _mm_mul_ps(ssepyi, ssex)), _mm_mul_ps(sse_Iu, _mm_mul_ps(ssepxi, ssey)));

            // Interaction matrix for the light affine model
            sse_L[6] = sse_Ia;
            sse_L[7] = sse_1;

            // Update lower triangular part of the system
            for (int k = 0; k < 8; k++)
            {
               for (int l = 0; l <= k; l++)
               {
                  sse_LtL[k][l] = _mm_add_ps(sse_LtL[k][l], _mm_mul_ps(sse_L[k], sse_L[l]));
               }
               sse_Lte[k] = _mm_add_ps(sse_Lte[k], _mm_mul_ps(sse_L[k], sse_Id));
            }
         }

         ptr_Iu+=4;
         ptr_Iv+=4;
         ptr_Id+=4;
         ptr_Ia+=4;
         ptr_Im+=4;

         sse_ur = _mm_add_ps ( sse_ur, sse_4 );
      }

      // Transfer lower triangular part of the system from SSE to CPU
      for ( int k = 0; k < 8; k++ )
      {
         // Aligned buffer of 4 32 bits float (4 * 32 = 128) to write __m128 registers
         union ssevector buffer;

         for ( int l = 0; l <= k; l++ )
         {
            sse_LtL[k][l] = _mm_hadd_ps(sse_LtL[k][l], sse_LtL[k][l]);
            sse_LtL[k][l] = _mm_hadd_ps(sse_LtL[k][l], sse_LtL[k][l]);
            _mm_store1_ps(buffer.tab, sse_LtL[k][l]);
            LtL_data[k][l] += buffer.tab[0];
         }

         sse_Lte[k] = _mm_hadd_ps(sse_Lte[k], sse_Lte[k]);
         sse_Lte[k] = _mm_hadd_ps(sse_Lte[k], sse_Lte[k]);
         _mm_store1_ps(buffer.tab, sse_Lte[k]);
         Lte_data[k] += buffer.tab[0];
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

   return error;
}
