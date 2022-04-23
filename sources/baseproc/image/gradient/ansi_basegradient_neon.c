//============================================================================
//
//    OPENROX   : File ansi_basegradient_neon.c
//
//    Contents  : Implementation of basegradient module
//                with NEON optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_basegradient.h"

#include <system/vectorisation/neon.h>

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

   // Use cols-2 since we do not compute borders
   int cols4 = (cols - 2)/4;
   if ( (cols - 2) %4 ) cols4++;

   float32x4_t neon_4 = vdupq_n_f32(4);
   float32x4_t neon_half = vdupq_n_f32(0.5);
   float32x4_t neon_cols = vdupq_n_f32(cols);

   Rox_Neon_Float uneon_u;
   float32x4_t neon_u;
   uint32x4_t mask;

   uneon_u.tab[0] = 0;
   uneon_u.tab[1] = 1;
   uneon_u.tab[2] = 2;
   uneon_u.tab[3] = 3;

   // Set first and last rows to zero (should we use memset ?)
   for ( int u = 0; u < cols; u++)
   {
      Gm_data[0][u] = 0;
      Gm_data[rows-1][u] = 0;
   }

   for ( int v = 1; v < rows - 1; v++)
   {
      unsigned int * ptr_Im  = &Im_data[v][1];
      unsigned int * ptr_Iml = &Im_data[v][0];
      unsigned int * ptr_Imr = &Im_data[v][2];
      unsigned int * ptr_Imt = &Im_data[v-1][1];
      unsigned int * ptr_Imb = &Im_data[v+1][1];

      float * ptr_It = &I_data[v-1][1];
      float * ptr_Ib = &I_data[v+1][1];
      float * ptr_Il = &I_data[v][0];
      float * ptr_Ir = &I_data[v][2];

      unsigned int * ptr_Gm = &Gm_data[v][1];

      float * ptr_Iu = &Iu_data[v][1];
      float * ptr_Iv = &Iv_data[v][1];

      neon_u = uneon_u.ssetype;

      for ( int u = 0; u < cols4; u++)
      {
         mask = vld1q_u32((unsigned int*)ptr_Im);
         mask = vandq_u32(mask, vcltq_f32(neon_u, neon_cols));
         mask = vandq_u32(mask, vld1q_u32((unsigned int*)ptr_Iml));
         mask = vandq_u32(mask, vld1q_u32((unsigned int*)ptr_Imr));
         mask = vandq_u32(mask, vld1q_u32((unsigned int*)ptr_Imt));
         mask = vandq_u32(mask, vld1q_u32((unsigned int*)ptr_Imb));

         float32x4_t top = vld1q_f32(ptr_It);
         float32x4_t bottom = vld1q_f32(ptr_Ib);
         float32x4_t left = vld1q_f32(ptr_Il);
         float32x4_t right = vld1q_f32(ptr_Ir);

         float32x4_t neon_Iu = vsubq_f32(right, left);
         neon_Iu = vmulq_f32(neon_Iu, neon_half);

         float32x4_t neon_Iv = vsubq_f32(bottom, top);
         neon_Iv = vmulq_f32(neon_Iv, neon_half);

         vst1q_u32(ptr_Gm, mask);
         vst1q_f32(ptr_Iu, neon_Iu);
         vst1q_f32(ptr_Iv, neon_Iv);

         ptr_Im  += 4;
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

         neon_u = vaddq_f32 ( neon_u, neon_4 );
      }

      Gm_data[v][0] = 0;
      Gm_data[v][cols-1]=0;
   }

   return error;
}

// TO BE OPTIMISED
int rox_ansi_array2d_float_basegradient_nomask (
   float ** Iu_data,
   float ** Iv_data,
   float ** I_data,
   int rows,
   int cols
)
{
   int error = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel
   {
   #pragma omp for schedule(dynamic)
#endif

   for (int i = 0; i < rows; i++)
   {
      int ni = i + 1;
      int pi = i - 1;

      for (int j = 0; j < cols; j++)
      {
         if (i == 0 || i == rows - 1) continue;
         if (j == 0 || j == cols - 1) continue;

         int nj = j + 1;
         int pj = j - 1;

         Iu_data[i][j] = 0.5f*(I_data[i][nj] - I_data[i][pj]);
         Iv_data[i][j] = 0.5f*(I_data[ni][j] - I_data[pi][j]);
      }
   }

#ifdef ROX_USES_OPENMP
   }
#endif

   return error;
}