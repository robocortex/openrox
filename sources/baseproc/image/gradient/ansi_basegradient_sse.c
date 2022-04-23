//============================================================================
//
//    OPENROX   : File ansi_basegradient_sse.c
//
//    Contents  : Implementation of basegradient module 
//                with SSE optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_basegradient.h"

#include <system/vectorisation/sse.h>

int rox_ansi_array2d_float_basegradient (
   float ** Iu_data,
   float ** Iv_data,
   unsigned int ** Gm_data,
   float ** I_data,
   unsigned int ** Im_data,
   int rows,
   int cols
)
{
   int error = 0;

   int cols4 = (cols - 2)/4;
   if ( (cols - 2) %4) cols4++;

   __m128 sse_4 = _mm_set_ps1(4.0f);
   __m128 ssehalf = _mm_set_ps1(0.5f);
   __m128 ssecols = _mm_set_ps1((float) cols);

   // Set first and last row gradient mask to 0
   // The gradient of the borders is not computed
   for ( int j = 0; j < cols; j++)
   {
      Gm_data[0][j] = 0;
      Gm_data[rows-1][j] = 0;
   }

   for ( int v = 1; v < rows - 1; v++)
   {
      float * ptr_Iu = &Iu_data[v][1];
      float * ptr_Iv = &Iv_data[v][1];

      unsigned int * ptr_Gm = &Gm_data[v][1];

      unsigned int * ptr_Im  = &Im_data[v][1];
      unsigned int * ptr_Iml = &Im_data[v][0];
      unsigned int * ptr_Imr = &Im_data[v][2];
      unsigned int * ptr_Imt = &Im_data[v-1][1];
      unsigned int * ptr_Imb = &Im_data[v+1][1];

      float * ptr_It = &I_data[v-1][1];
      float * ptr_Ib = &I_data[v+1][1];
      float * ptr_Il = &I_data[v][0];
      float * ptr_Ir = &I_data[v][2];

      __m128 sse_u = _mm_set_ps(4,3,2,1);

      for ( int u = 0; u < cols4; u++)
      {
         __m128 mask = _mm_loadu_ps( (float*) ptr_Im);

         mask = _mm_and_ps ( mask, _mm_cmplt_ps ( sse_u, ssecols ) );
         mask = _mm_and_ps ( mask, _mm_loadu_ps ( (float*) ptr_Iml ) );
         mask = _mm_and_ps ( mask, _mm_loadu_ps ( (float*) ptr_Imr ) );
         mask = _mm_and_ps ( mask, _mm_loadu_ps ( (float*) ptr_Imt ) );
         mask = _mm_and_ps ( mask, _mm_loadu_ps ( (float*) ptr_Imb ) );

         __m128 top = _mm_loadu_ps(ptr_It);
         __m128 bottom = _mm_loadu_ps(ptr_Ib);
         __m128 left = _mm_loadu_ps(ptr_Il);
         __m128 right = _mm_loadu_ps(ptr_Ir);

         __m128 sse_Iu = _mm_sub_ps(right, left);
         sse_Iu = _mm_mul_ps(sse_Iu, ssehalf);

         __m128 sse_Iv = _mm_sub_ps(bottom, top);
         sse_Iv = _mm_mul_ps(sse_Iv, ssehalf);

         _mm_storeu_ps((float*)ptr_Gm, mask);
         _mm_storeu_ps(ptr_Iu, sse_Iu);
         _mm_storeu_ps(ptr_Iv, sse_Iv);

         ptr_Im += 4;
         ptr_Iml += 4;
         ptr_Imr += 4;
         ptr_Imt += 4;
         ptr_Imb += 4;

         ptr_Il += 4;
         ptr_Ir += 4;
         ptr_It += 4;
         ptr_Ib += 4;

         ptr_Gm += 4;

         ptr_Iu += 4;
         ptr_Iv += 4;

         sse_u = _mm_add_ps ( sse_u, sse_4 );
      }

      Gm_data[v][0] = 0;
      Gm_data[v][cols-1] = 0;
   }

   return error;
}


int rox_ansi_array2d_float_basegradient_nomask (
   float ** Iu_data,
   float ** Iv_data,
   float ** I_data,
   int rows,
   int cols
)
{
   int error = 0;
   int cols4 = (cols - 2)/4;
   if ((cols - 2)%4) cols4++;

   __m128 sse_4 = _mm_set_ps1(4);
   __m128 ssehalf = _mm_set_ps1(0.5);

   for ( int v = 1; v < rows - 1; v++)
   {
      float * ptr_It = &I_data[v-1][1];
      float * ptr_Ib = &I_data[v+1][1];
      float * ptr_Il = &I_data[v][0];
      float * ptr_Ir = &I_data[v][2];

      float * ptr_Iu = &Iu_data[v][1];
      float * ptr_Iv = &Iv_data[v][1];

      __m128 sse_u = _mm_set_ps(4,3,2,1);

      for ( Rox_Sint u = 0; u < cols4; u++)
      {
         __m128 top = _mm_loadu_ps(ptr_It);
         __m128 bottom = _mm_loadu_ps(ptr_Ib);
         __m128 left = _mm_loadu_ps(ptr_Il);
         __m128 right = _mm_loadu_ps(ptr_Ir);

         // Compute difference sse_Iu = rigth - left
         __m128 sse_Iu = _mm_sub_ps(right, left);
         // Divide difference sse_Iuby 2.0
         sse_Iu = _mm_mul_ps(sse_Iu, ssehalf);

         // Compute difference sse_Iu = bootom - top
         __m128 sse_Iv = _mm_sub_ps(bottom, top);
         // Divide differenceby 2.0
         sse_Iv = _mm_mul_ps(sse_Iv, ssehalf);

         _mm_storeu_ps(ptr_Iu, sse_Iu);
         _mm_storeu_ps(ptr_Iv, sse_Iv);

         // Increment pointers
         ptr_Il += 4;
         ptr_Ir += 4;
         ptr_It += 4;
         ptr_Ib += 4;

         ptr_Iu += 4;
         ptr_Iv += 4;

         // Increment u coordinates
         sse_u = _mm_add_ps(sse_u, sse_4);
      }
   }
   return error;
}
