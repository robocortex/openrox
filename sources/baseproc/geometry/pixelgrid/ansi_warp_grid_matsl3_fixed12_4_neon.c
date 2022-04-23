//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matsl3_fixed12_4_neon.c
//
//    Contents  : Implementation of warp_grid_matsl3_fixed12_4 module 
//                with NEON optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_warp_grid_matsl3_fixed12_4.h"
#include <system/arch/platform_neon.h>
#include <float.h>

int rox_ansi_warp_grid_sl3_fixed12_4 (
   short ** grid_u_data, 
   short ** grid_v_data,
   int rows,
   int cols,
   double ** H_data
)
{
   int error = 0;

   int cols4 = cols / 4;
   if (cols % 4) cols4++;

   uint32x4_t maskinf, masksup, mask;
   //int16x4_t neon_nuf, neon_nvf;
   //int16x4x2_t neon_xy;
   Rox_Neon_Float uneon_j;

   float32x4_t neon_1 = vdupq_n_f32(1);
   float32x4_t neon_4 = vdupq_n_f32(4);
   float32x4_t neon_16 = vdupq_n_f32(16);

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

      float32x4_t neon_ru = vdupq_n_f32(ru);
      float32x4_t neon_rv = vdupq_n_f32(rv);
      float32x4_t neon_rw = vdupq_n_f32(rw);
      float32x4_t neon_j = uneon_j.ssetype;

      // ptrxy = mxy[i];
      short * ptr_u = grid_u_data[i];
      short * ptr_v = grid_v_data[i];
      
      for ( int j = 0; j < cols4; j++)
      {
         float32x4_t neon_nu = vaddq_f32(vmulq_f32(neon_h1, neon_j), neon_ru);
         float32x4_t neon_nv = vaddq_f32(vmulq_f32(neon_h2, neon_j), neon_rv);
         float32x4_t neon_nw = vaddq_f32(vmulq_f32(neon_h3, neon_j), neon_rw);

         maskinf = vcltq_f32(neon_nw, neon_meps);
         masksup = vcgtq_f32(neon_nw, neon_eps);
         mask = vorrq_u32(maskinf, masksup);

         // Set 1 to w if mask is 0
         neon_nw = vbslq_f32(mask, neon_nw, neon_1);

         float32x4_t neon_inw = vrecpeq_f32(neon_nw);
         neon_inw = vmulq_f32(neon_inw, vrecpsq_f32(neon_nw, neon_inw));

         neon_nu = vmulq_f32(neon_nu, neon_inw);
         neon_nv = vmulq_f32(neon_nv, neon_inw);

         neon_nu = vmulq_f32(neon_nu, neon_16);
         neon_nv = vmulq_f32(neon_nv, neon_16);
         
         int16x4_t neon_nuf = vqmovn_s32(vcvtq_s32_f32(neon_nu));
         int16x4_t neon_nvf = vqmovn_s32(vcvtq_s32_f32(neon_nv));

         vst1_s16( (short*) ptr_u, neon_nuf); 
         vst1_s16( (short*) ptr_v, neon_nvf); 

         // neon_xy.val[0] = neon_nuf;
         // neon_xy.val[1] = neon_nvf;

         // vst2_s16((Rox_Sshort*)ptrxy, neon_xy); 

         // ptrxy += 4;
         ptr_u +=4;
         ptr_v +=4;

         neon_j = vaddq_f32(neon_j, neon_4);
      }
   }

   return error;
}

#ifdef old

Rox_ErrorCode rox_warp_grid_sl3_fixed12_4 (
   Rox_Array2D_Point2D_Sshort grid, 
   const Rox_MatSL3 homography 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!grid || !homography) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_matsl3_check_size ( homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &H_data, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Point2D_Sshort * mxy = NULL, ptrxy = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &mxy, grid );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols, rows;
   error = rox_array2d_point2d_sshort_get_size ( &rows, &cols, grid );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifndef integrated
   Rox_Sint cols4 = cols / 4;
   if (cols % 4) cols4++;

   uint32x4_t maskinf, masksup, mask;
   //int16x4_t neon_nuf, neon_nvf;
   int16x4x2_t neon_xy;
   Rox_Neon_Float uneon_j;

   float32x4_t neon_1 = vdupq_n_f32(1);
   float32x4_t neon_4 = vdupq_n_f32(4);
   float32x4_t neon_16 = vdupq_n_f32(16);

   float32x4_t neon_h1   = vdupq_n_f32(H_data[0][0]);
   float32x4_t neon_h2   = vdupq_n_f32(H_data[1][0]);
   float32x4_t neon_h3   = vdupq_n_f32(H_data[2][0]);
   float32x4_t neon_meps = vdupq_n_f32(-FLT_EPSILON);
   float32x4_t neon_eps  = vdupq_n_f32(FLT_EPSILON);

   uneon_j.tab[0] = 0;
   uneon_j.tab[1] = 1;
   uneon_j.tab[2] = 2;
   uneon_j.tab[3] = 3;

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Double v = (Rox_Double) i;
      Rox_Double ru = H_data[0][1] * v + H_data[0][2];
      Rox_Double rv = H_data[1][1] * v + H_data[1][2];
      Rox_Double rw = H_data[2][1] * v + H_data[2][2];

      float32x4_t neon_ru = vdupq_n_f32(ru);
      float32x4_t neon_rv = vdupq_n_f32(rv);
      float32x4_t neon_rw = vdupq_n_f32(rw);
      float32x4_t neon_j = uneon_j.ssetype;

      ptrxy = mxy[i];

      for ( Rox_Sint j = 0; j < cols4; j++)
      {
         float32x4_t neon_nu = vaddq_f32(vmulq_f32(neon_h1, neon_j), neon_ru);
         float32x4_t neon_nv = vaddq_f32(vmulq_f32(neon_h2, neon_j), neon_rv);
         float32x4_t neon_nw = vaddq_f32(vmulq_f32(neon_h3, neon_j), neon_rw);

         maskinf = vcltq_f32(neon_nw, neon_meps);
         masksup = vcgtq_f32(neon_nw, neon_eps);
         mask = vorrq_u32(maskinf, masksup);

         // Set 1 to w if mask is 0
         neon_nw = vbslq_f32(mask, neon_nw, neon_1);

         float32x4_t neon_inw = vrecpeq_f32(neon_nw);
         neon_inw = vmulq_f32(neon_inw, vrecpsq_f32(neon_nw, neon_inw));

         neon_nu = vmulq_f32(neon_nu, neon_inw);
         neon_nv = vmulq_f32(neon_nv, neon_inw);

         neon_nu = vmulq_f32(neon_nu, neon_16);
         neon_nv = vmulq_f32(neon_nv, neon_16);
         
         int16x4_t neon_nuf = vqmovn_s32(vcvtq_s32_f32(neon_nu));
         int16x4_t neon_nvf = vqmovn_s32(vcvtq_s32_f32(neon_nv));

         neon_xy.val[0] = neon_nuf;
         neon_xy.val[1] = neon_nvf;

         vst2_s16( (Rox_Sshort*) ptrxy, neon_xy); 

         ptrxy += 4;

         neon_j = vaddq_f32 ( neon_j, neon_4 );
      }
   }
#else
   error = rox_neon_warp_grid_sl3_fixed12_4 ( grid_u_data, grid_v_data, rows, cols, H_data );
   ROX_ERROR_CHECK_TERMINATE ( error );
#endif

function_terminate:
   return error;
}

#endif