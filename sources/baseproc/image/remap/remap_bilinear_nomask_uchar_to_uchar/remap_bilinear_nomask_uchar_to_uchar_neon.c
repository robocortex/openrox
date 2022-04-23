//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_uchar_to_uchar_neon.c
//
//    Contents  : Implementation of remap_bilinear_nomask_uchar_to_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_nomask_uchar_to_uchar.h"

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_bilinear_nomask_uchar_to_uchar ( 
   Rox_Image output, 
   const Rox_Image input, 
   const Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input || !grid)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width, height;
   error = rox_array2d_uchar_get_size(&height, &width, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint inwidth, inheight;
   error = rox_array2d_uchar_get_size(&inheight, &inwidth, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size ( grid, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&out, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** in = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&in, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < height; i++)
   {
      for (Rox_Uint j = 0; j < width; j++)
      {
         out[i][j] = 255;

         Rox_Sint cx = (Rox_Sint) grid_u_data[i][j];
         Rox_Sint cy = (Rox_Sint) grid_v_data[i][j];

         Rox_Double ix = cx;
         Rox_Double iy = cy;

         if ( (cx < 0) || (cy < 0) ) continue;
         if ( (cx > inwidth - 1) || (cy > inheight - 1) ) continue;

         // Added test for borders special case
         if ( (cx == inwidth - 1) || (cy == inheight - 1) )
         {
            out[i][j] = in[cy][cx];
            continue;
         }

         Rox_Double dx = grid_u_data[i][j] - ix;
         Rox_Double dy = grid_v_data[i][j] - iy;

         Rox_Double b1 = (Rox_Double)in[cy][cx];
         Rox_Double b2 = (Rox_Double)in[cy][cx+1] - b1;
         Rox_Double b3 = (Rox_Double)in[cy+1][cx] - b1;
         Rox_Double b4 = b1 + (Rox_Double)in[cy+1][cx+1] - (Rox_Double)in[cy+1][cx] - (Rox_Double)in[cy][cx+1];

         out[i][j] = b1 + b2*dx + b3*dy + b4*dx*dy;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_remap_bilinear_nomask_uchar_to_uchar_fixed ( Rox_Image output, Rox_Image input, Rox_Array2D_Point2D_Sshort map )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input || !map) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint inwidth = 0, inheight = 0;
   error = rox_array2d_uchar_get_size(&inheight, &inwidth, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_check_size(map, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&out, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** in = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&in, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort * mxy = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &mxy, map);
   ROX_ERROR_CHECK_TERMINATE ( error );


   int16x8_t   neonmask;
   int16x8_t   neoncx, neoncy, neondx, neondy, neonv1, neonv2, neonv3, neonv4;
   int16x8_t   neonb1, neonb2, neonb3, neonb4, neonout;
   int16x8x2_t neonxy;

   uint16x8_t neonuout;

   int16x8_t neonmask_border = vdupq_n_s16(~0);
   int16x8_t neon0 = vdupq_n_s16(0);
   int16x8_t neon15 = vdupq_n_s16(15);
   int16x8_t neon255 = vdupq_n_u16(255);
   int16x8_t neonwidth = vdupq_n_s16(inwidth - 1);
   int16x8_t neonheight = vdupq_n_s16(inheight - 1);

   Rox_Sshort vmask[8], vx[8], vy[8],vv1[8],vv2[8],vv3[8],vv4[8];


   Rox_Sint width8 = width / 8;
   if (width8 * 8 < width)
   {
      for (Rox_Uint j = 0; j < 8; j++)
      {
         if (width8 * 8 + j < width)
         {
            vmask[j] = ~0;
         }
         else
         {
            vmask[j] = 0;
         }
      }
      neonmask_border = vld1q_s16(vmask);

      width8++;
   }

   for (Rox_Uint i = 0; i < height; i++)
   {
      Rox_Uchar * rowout = out[i];
      Rox_Point2D_Sshort rowxy = mxy[i];

      for (Rox_Uint j = 0; j < width8; j++)
      {
         neonxy = vld2q_s16((int16_t *)rowxy);
         neoncx = vshrq_n_s16(neonxy.val[0], 4);
         neoncy = vshrq_n_s16(neonxy.val[1], 4);
         neondx = vandq_s16(neonxy.val[0], neon15);
         neondy = vandq_s16(neonxy.val[1], neon15);

         neonmask = vreinterpretq_s16_u16(vcgeq_s16(neoncx, neon0));
         neonmask = vandq_s16(neonmask, vreinterpretq_s16_u16(vcgeq_s16(neoncy, neon0)));
         neonmask = vandq_s16(neonmask, vreinterpretq_s16_u16(vcltq_s16(neoncx, neonwidth)));
         neonmask = vandq_s16(neonmask, vreinterpretq_s16_u16(vcltq_s16(neoncy, neonheight)));

         if (j == width8 - 1)
         {
            neonmask = vandq_s16(neonmask, neonmask_border);
         }

	      neoncx = vandq_s16(neoncx, neonmask);
	      neoncy = vandq_s16(neoncy, neonmask);

         vst1q_s16(vx, neoncx);
         vst1q_s16(vy, neoncy);

         for (Rox_Uint k = 0; k < 8; k++)
         {
            Rox_Sint cx = vx[k];
            Rox_Sint cy = vy[k];

	         vv1[k] = in[cy][cx];
	         vv2[k] = in[cy][cx + 1];
	         vv3[k] = in[cy + 1][cx];
	         vv4[k] = in[cy + 1][cx + 1];
         }

         neonv1 = vld1q_s16(vv1);
         neonv2 = vld1q_s16(vv2);
         neonv3 = vld1q_s16(vv3);
         neonv4 = vld1q_s16(vv4);

         neonb1 = neonv1;
         neonb2 = vsubq_s16(neonv2, neonv1);
         neonb3 = vsubq_s16(neonv3, neonv1);
         neonb4 = vsubq_s16(vsubq_s16(vaddq_s16(neonv1, neonv4), neonv3), neonv2);

         neonb1 = vshlq_n_s16(neonb1, 4);
         neonb2 = vmulq_s16(neonb2, neondx);
         neonb3 = vmulq_s16(neonb3, neondy);
         neonb1 = vshlq_n_s16(vaddq_s16(vaddq_s16(neonb2, neonb3), neonb1), 4);
         neonb4 = vmulq_s16(vmulq_s16(neonb4, neondx), neondy);

         neonout = vaddq_s16(neonb1, neonb4);

         neonuout = vreinterpretq_u16_s16(neonout);
         neonuout = vshrq_n_u16(neonuout, 8);
         neonuout = vandq_u16(neonuout, neon255);

         vst1_u8(rowout, vmovn_u16(neonuout));

         rowout += 8;
         rowxy += 8;
      }
   }

function_terminate:
   return error;
}
