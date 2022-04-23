//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matsl3_sse.c
//
//    Contents  : Implementation of warp_grid_matsl3 module with SSE optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_warp_grid_matsl3.h"

#include <float.h>
#include <system/vectorisation/sse.h>

int rox_ansi_warp_grid_sl3_float (
   float ** grid_u_data, 
   float ** grid_v_data,
   int rows,
   int cols,
   double ** H_data
)
{
   int error = 0;
   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   __m128 sse_1 = _mm_set_ps1(1);
   __m128 sse_4 = _mm_set_ps1(4);

   __m128 sse_meps = _mm_set_ps1(-FLT_EPSILON);
   __m128 sse_eps = _mm_set_ps1(FLT_EPSILON);

   __m128 sse_H00 = _mm_set_ps1((float) H_data[0][0]);
   __m128 sse_H10 = _mm_set_ps1((float) H_data[1][0]);
   __m128 sse_H20 = _mm_set_ps1((float) H_data[2][0]);

   __m128 sse_H01 = _mm_set_ps1((float) H_data[0][1]);
   __m128 sse_H11 = _mm_set_ps1((float) H_data[1][1]);
   __m128 sse_H21 = _mm_set_ps1((float) H_data[2][1]);

   __m128 sse_H02 = _mm_set_ps1((float) H_data[0][2]);
   __m128 sse_H12 = _mm_set_ps1((float) H_data[1][2]);
   __m128 sse_H22 = _mm_set_ps1((float) H_data[2][2]);

   for ( int v = 0; v < rows; v++)
   {
      float * ptr_u = grid_u_data[v];
      float * ptr_v = grid_v_data[v];

      __m128 sse_u = _mm_set_ps(3, 2, 1, 0);
      __m128 sse_v = _mm_set_ps1((float) v);

      __m128 sse_ru = _mm_add_ps(_mm_mul_ps(sse_H01, sse_v), sse_H02);
      __m128 sse_rv = _mm_add_ps(_mm_mul_ps(sse_H11, sse_v), sse_H12);
      __m128 sse_rw = _mm_add_ps(_mm_mul_ps(sse_H21, sse_v), sse_H22);

      for ( int u = 0; u < cols4; u++)
      {
         __m128 sse_nu = _mm_add_ps(_mm_mul_ps(sse_H00, sse_u), sse_ru);
         __m128 sse_nv = _mm_add_ps(_mm_mul_ps(sse_H10, sse_u), sse_rv);
         __m128 sse_nw = _mm_add_ps(_mm_mul_ps(sse_H20, sse_u), sse_rw);

         // Divide if not 0
         __m128 maskinf = _mm_cmplt_ps(sse_nw, sse_meps);
         __m128 masksup = _mm_cmpgt_ps(sse_nw, sse_eps);
         __m128 mask = _mm_or_ps(maskinf, masksup);
         sse_nw = _mm_blendv_ps(sse_1, sse_nw, mask);
         __m128 sse_inw = _mm_div_ps(sse_1, sse_nw);

         // Project
         sse_nu = _mm_mul_ps(sse_nu, sse_inw);
         sse_nv = _mm_mul_ps(sse_nv, sse_inw);

         // Get data into CPU memory
         _mm_storeu_ps(ptr_u, sse_nu);
         _mm_storeu_ps(ptr_v, sse_nv);

         ptr_u+=4;
         ptr_v+=4;

         sse_u = _mm_add_ps(sse_u, sse_4);
      }
   }

   return error;
}
