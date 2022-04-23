//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matse3_z_sse.c
//
//    Contents  : Implementation of warp_grid_matse3_z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_warp_grid_matse3_z.h"

#include <float.h>
#include <system/vectorisation/sse.h>

int rox_ansi_warp_grid_float_matse3_z_float (
   float ** grid_u_data,
   float ** grid_v_data,
   unsigned int ** grid_mask_data,
   float ** Zr_data,
   int rows,
   int cols,
   double ** cQr_data
)
{
   int error = 0;

   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   __m128 sse_epsilon = _mm_set_ps1(FLT_EPSILON);
   __m128 sse_1 = _mm_set_ps1(1);
   __m128 sse_4 = _mm_set_ps1(4);

   __m128 Q11 = _mm_set_ps1((float) cQr_data[0][0]);
   __m128 Q12 = _mm_set_ps1((float) cQr_data[0][1]);
   __m128 Q13 = _mm_set_ps1((float) cQr_data[0][2]);
   __m128 Q14 = _mm_set_ps1((float) cQr_data[0][3]);

   __m128 Q21 = _mm_set_ps1((float) cQr_data[1][0]);
   __m128 Q22 = _mm_set_ps1((float) cQr_data[1][1]);
   __m128 Q23 = _mm_set_ps1((float) cQr_data[1][2]);

   __m128 Q24 = _mm_set_ps1((float) cQr_data[1][3]);
   __m128 Q31 = _mm_set_ps1((float) cQr_data[2][0]);
   __m128 Q32 = _mm_set_ps1((float) cQr_data[2][1]);
   __m128 Q33 = _mm_set_ps1((float) cQr_data[2][2]);
   __m128 Q34 = _mm_set_ps1((float) cQr_data[2][3]);

   for ( int v = 0; v < rows; v++)
   {
      __m128 sse_u = _mm_set_ps(3, 2, 1, 0);
      __m128 sse_v = _mm_set_ps1((float)v);

      unsigned int * ptrm = grid_mask_data[v];
      float * ptrd = Zr_data[v];
      float * ptr_u = grid_u_data[v];
      float * ptr_v = grid_v_data[v];

      for ( int u = 0; u < cols4; u++)
      {
         __m128 sse_Zr = _mm_loadu_ps(ptrd);
         // sse_uZr = sse_u * sse_Zr
         __m128 sse_uZr = _mm_mul_ps(sse_u, sse_Zr);
         // sse_vZr = sse_v * sse_Zr
         __m128 sse_vZr = _mm_mul_ps(sse_v, sse_Zr);

         // sse_pu = Q11 * sse_uZr
         __m128 sse_pu = _mm_mul_ps(Q11, sse_uZr);
         // sse_pu = sse_pu + Q12 * sse_vZr
         sse_pu = _mm_add_ps(sse_pu, _mm_mul_ps(Q12, sse_vZr));
         // sse_pu = sse_pu + Q13 * sse_Zr
         sse_pu = _mm_add_ps(sse_pu, _mm_mul_ps(Q13, sse_Zr));
         // sse_pu = sse_pu + Q14
         sse_pu = _mm_add_ps(sse_pu, Q14);
         // sse_pu = Q11 * sse_uZr + Q12 * sse_vZr + Q13 * sse_Zr + Q14

         // sse_pv = Q21 * sse_uZr
         __m128 sse_pv = _mm_mul_ps(Q21, sse_uZr);
         // sse_pv = sse_pv + Q22 * sse_vZr
         sse_pv = _mm_add_ps(sse_pv, _mm_mul_ps(Q22, sse_vZr));
         // sse_pv = sse_pv + Q23 * sse_Zr
         sse_pv = _mm_add_ps(sse_pv, _mm_mul_ps(Q23, sse_Zr));
         // sse_pv = sse_pv + Q24
         sse_pv = _mm_add_ps(sse_pv, Q24);
         // sse_pv = Q21 * sse_uZr + Q22 * sse_vZr + Q23 * sse_Zr + Q24

         // sse_pw = Q31 * sse_uZr
         __m128 sse_pw = _mm_mul_ps(Q31, sse_uZr);
         // sse_pw = sse_pw + Q32 * sse_vZr
         sse_pw = _mm_add_ps(sse_pw, _mm_mul_ps(Q32, sse_vZr));
         // sse_pw = sse_pw + Q33 * sse_Zr
         sse_pw = _mm_add_ps(sse_pw, _mm_mul_ps(Q33, sse_Zr));
         // sse_pw = sse_pw + Q34
         sse_pw = _mm_add_ps(sse_pw, Q34);
         // sse_pw = Q31 * sse_uZr + Q32 * sse_vZr + Q33 * sse_Zr + Q34

         // Test if sse_pw is equal to 0
         __m128 sse_mask = _mm_cmpgt_ps(sse_pw, sse_epsilon);
         // Test if sse_u is equal to cols ???
         // sse_mask = _mm_and_ps(sse_mask, _mm_cmplt_ps(sse_u, sse_cols));

         // If mask = 1 then sse_pw = sse_pw
         // If mask = 0 then sse_pw = sse_1
         sse_pw = _mm_blendv_ps(sse_1, sse_pw, sse_mask);
         // sse_pwi = 1/sse_pw (no need to test if sse_pw = 0 since it has been already tested)
         __m128 sse_pwi = _mm_div_ps(sse_1, sse_pw);
         // sse_pu = sse_pu / sse_pw
         sse_pu = _mm_mul_ps(sse_pu, sse_pwi);
         // sse_pv = sse_pv / sse_pw
         sse_pv = _mm_mul_ps(sse_pv, sse_pwi);

         _mm_storeu_ps(ptr_u, sse_pu);
         _mm_storeu_ps(ptr_v, sse_pv);

         _mm_storeu_ps((float*) ptrm, sse_mask);

         ptr_u+=4;
         ptr_v+=4;

         ptrm += 4;
         ptrd += 4;

         // Add (4,4,4,4) to sse_u
         sse_u = _mm_add_ps(sse_u, sse_4);
      }
   }

   return error;
}
