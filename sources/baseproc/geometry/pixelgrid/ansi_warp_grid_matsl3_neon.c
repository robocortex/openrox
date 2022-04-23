//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matsl3_neon.c
//
//    Contents  : Implementation of warp_grid_matsl3 module
//                with NEON optimisation
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
#include <system/vectorisation/neon.h>

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

   float32x4_t neon_ru, neon_rv, neon_rw, neon_j;
   float32x4_t neon_nu, neon_nv, neon_nw, neon_inw;
   uint32x4_t maskinf, masksup, mask;
   //float32x4x2_t neon_xy;
   Rox_Neon_Float uneon_j;

   float32x4_t neon_1 = vdupq_n_f32(1);
   float32x4_t neon_4 = vdupq_n_f32(4);
   float32x4_t neon_h1 = vdupq_n_f32(H_data[0][0]);
   float32x4_t neon_h2 = vdupq_n_f32(H_data[1][0]);
   float32x4_t neon_h3 = vdupq_n_f32(H_data[2][0]);
   float32x4_t neon_meps = vdupq_n_f32(-FLT_EPSILON);
   float32x4_t neon_eps = vdupq_n_f32(FLT_EPSILON);

   uneon_j.tab[0] = 0;
   uneon_j.tab[1] = 1;
   uneon_j.tab[2] = 2;
   uneon_j.tab[3] = 3;

   for ( int i = 0; i < rows; i++)
   {
      double v = (double) i;
      double ru = H_data[0][1] * v + H_data[0][2];
      double rv = H_data[1][1] * v + H_data[1][2];
      double rw = H_data[2][1] * v + H_data[2][2];

      neon_ru = vdupq_n_f32(ru);
      neon_rv = vdupq_n_f32(rv);
      neon_rw = vdupq_n_f32(rw);
      neon_j = uneon_j.ssetype;

      // ptrxy = mxy[i];
      float * ptr_u = grid_u_data[i];
      float * ptr_v = grid_v_data[i];

      for ( int j = 0; j < cols4; j++)
      {
         neon_nu = vaddq_f32(vmulq_f32(neon_h1, neon_j), neon_ru);
         neon_nv = vaddq_f32(vmulq_f32(neon_h2, neon_j), neon_rv);
         neon_nw = vaddq_f32(vmulq_f32(neon_h3, neon_j), neon_rw);

         maskinf = vcltq_f32(neon_nw, neon_meps);
         masksup = vcgtq_f32(neon_nw, neon_eps);
         mask = vorrq_u32(maskinf, masksup);

         // Set 1 to w if mask is 0
         neon_nw = vbslq_f32(mask, neon_nw, neon_1);

         neon_inw = vrecpeq_f32(neon_nw);
         neon_inw = vmulq_f32(neon_inw, vrecpsq_f32(neon_nw, neon_inw));

         neon_nu = vmulq_f32(neon_nu, neon_inw);
         neon_nv = vmulq_f32(neon_nv, neon_inw);

         vst1q_f32( (float*) ptr_u, neon_nu); 
         vst1q_f32( (float*) ptr_v, neon_nv); 

         // neon_xy.val[0] = neon_nu;
         // neon_xy.val[1] = neon_nv;

         // vst2q_f32( (float*) ptrxy, neon_xy); 

         // ptrxy += 4;
         ptr_u +=4;
         ptr_v +=4;

         neon_j = vaddq_f32(neon_j, neon_4);
      }
   }

   return error;
}
