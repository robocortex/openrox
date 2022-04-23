//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_uint_to_uint.c
//
//    Contents  : Implementation of remap_bilinear_nomask_uint_to_uint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_nomask_uint_to_uint.h"

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_bilinear_nomask_uint (
   Rox_Array2D_Uint image_out, 
   const Rox_Array2D_Uint image_inp, 
   const Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_out || !image_inp || !grid) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error  = rox_array2d_uint_get_size(&height, &width, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint inwidth = 0, inheight = 0;
   error  = rox_array2d_uint_get_size(&inheight, &inwidth, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size(grid, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** image_out_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_out_data, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** image_inp_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_inp_data, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      for (Rox_Sint j = 0; j < width; j++)
      {
         image_out_data[i][j] = 0;

         Rox_Sint cx = (Rox_Sint) grid_u_data[i][j];
         Rox_Sint cy = (Rox_Sint) grid_v_data[i][j];

         Rox_Double ix = cx;
         Rox_Double iy = cy;

         if (cx < 0 || cy < 0) continue;
         if (cx >= inwidth - 1 || cy >= inheight - 1) continue;
         
         // Added test for borders special case
         if ( (cx == inwidth - 1) || (cy == inheight - 1) )
         {
            image_out_data[i][j] = image_inp_data[cy][cx];
            continue;
         }

         Rox_Double dx = grid_u_data[i][j] - ix;
         Rox_Double dy = grid_v_data[i][j] - iy;

         Rox_Double b1 = (Rox_Double) image_inp_data[cy][cx];
         Rox_Double b2 = (Rox_Double) image_inp_data[cy][cx+1] - b1;
         Rox_Double b3 = (Rox_Double) image_inp_data[cy+1][cx] - b1;
         Rox_Double b4 = b1 + (Rox_Double) image_inp_data[cy+1][cx+1] - (Rox_Double) image_inp_data[cy+1][cx] - (Rox_Double) image_inp_data[cy][cx+1];

         image_out_data[i][j] = b1 + b2*dx + b3*dy + b4*dx*dy;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_remap_bilinear_nomask_rgba (
   Rox_Array2D_Uint image_out, 
   const Rox_Array2D_Uint image_inp, 
   const Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_out || !image_inp || !grid) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint width = 0, height = 0;
   error  = rox_array2d_uint_get_size(&height, &width, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint inwidth = 0, inheight = 0;
   error  = rox_array2d_uint_get_size(&inheight, &inwidth, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size(grid, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** image_out_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_out_data, image_out );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** image_inp_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_inp_data, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      for (Rox_Sint j = 0; j < width; j++)
      {
         image_out_data[i][j] = ~0;

         Rox_Sint cx = (Rox_Sint) grid_u_data[i][j];
         Rox_Sint cy = (Rox_Sint) grid_v_data[i][j];

         Rox_Double ix = cx;
         Rox_Double iy = cy;

         if (cx < 0 || cy < 0) continue;
         if (cx >= inwidth - 1 || cy >= inheight - 1) continue;

         Rox_Double dx = grid_u_data[i][j] - ix;
         Rox_Double dy = grid_v_data[i][j] - iy;

         // red
         Rox_Double r1 = (Rox_Double)(image_inp_data[cy][cx]   & 0x000000ff);
         Rox_Double r2 = (Rox_Double)(image_inp_data[cy][cx+1] & 0x000000ff) - r1;
         Rox_Double r3 = (Rox_Double)(image_inp_data[cy+1][cx] & 0x000000ff) - r1;
         Rox_Double r4 = r1 + (Rox_Double)(image_inp_data[cy+1][cx+1] & 0x000000ff) - (Rox_Double)(image_inp_data[cy+1][cx] & 0x000000ff) - (Rox_Double)(image_inp_data[cy][cx+1] & 0x000000ff);

         Rox_Uint r = (Rox_Uint) (r1 + r2*dx + r3*dy + r4*dx*dy);

         // green
         Rox_Double g1 = (Rox_Double)(image_inp_data[cy][cx] >> 8 & 0x000000ff);
         Rox_Double g2 = (Rox_Double)(image_inp_data[cy][cx+1] >> 8 & 0x000000ff) - g1;
         Rox_Double g3 = (Rox_Double)(image_inp_data[cy+1][cx] >> 8 & 0x000000ff) - g1;
         Rox_Double g4 = g1 + (Rox_Double)(image_inp_data[cy+1][cx+1] >> 8 & 0x000000ff) - (Rox_Double)(image_inp_data[cy+1][cx] >> 8 & 0x000000ff) - (Rox_Double)(image_inp_data[cy][cx+1] >> 8 & 0x000000ff);

         Rox_Uint g = (Rox_Uint) (g1 + g2*dx + g3*dy + g4*dx*dy);

         // blue
         Rox_Double b1 = (Rox_Double)(image_inp_data[cy][cx] >> 16 & 0x000000ff);
         Rox_Double b2 = (Rox_Double)(image_inp_data[cy][cx+1] >> 16 & 0x000000ff) - b1;
         Rox_Double b3 = (Rox_Double)(image_inp_data[cy+1][cx] >> 16 & 0x000000ff) - b1;
         Rox_Double b4 = b1 + (Rox_Double)(image_inp_data[cy+1][cx+1] >> 16 & 0x000000ff) - (Rox_Double)(image_inp_data[cy+1][cx] >> 16 & 0x000000ff) - (Rox_Double)(image_inp_data[cy][cx+1] >> 16 & 0x000000ff);

         Rox_Uint b = (Rox_Uint) (b1 + b2*dx + b3*dy + b4*dx*dy);

         // build output rgba
         image_out_data[i][j] = ((255 << 24) + (b << 16) + (g << 8) + r);
      }
   }

function_terminate:
   return error;
}

int rox_ansi_remap_bilinear_nomask_rgba_fixed (
   unsigned int ** image_out_data,
   unsigned int ** image_inp_data, 
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
)
{
   int error = 0;
   // TODO
   return error;
}

Rox_ErrorCode rox_remap_bilinear_nomask_rgba_fixed (
   Rox_Array2D_Uint image_out, 
   const Rox_Array2D_Uint image_inp, 
   const Rox_Array2D_Point2D_Sshort grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_out || !image_inp || !grid) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error  = rox_array2d_uint_get_size(&height, &width, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint inwidth = 0, inheight = 0;
   error  = rox_array2d_uint_get_size(&inheight, &inwidth, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_check_size(grid, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** image_out_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_out_data, image_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** image_inp_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_inp_data, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Point2D_Sshort * mxy = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &mxy, grid);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      for (Rox_Sint j = 0; j < width; j++)
      {
         Rox_Sshort val;

         image_out_data[i][j] = 255;

         Rox_Sint cx = mxy[i][j].u >> 4;
         Rox_Sint cy = mxy[i][j].v >> 4;

         Rox_Sshort dx = mxy[i][j].u & 15;
         Rox_Sshort dy = mxy[i][j].v & 15;

         if (cx < 0 || cy < 0) continue;
         if (cx >= inwidth - 1 || cy >= inheight - 1) continue;

         // red
         Rox_Sshort r1 = (image_inp_data[cy][cx] & 0x000000ff);//maxval = 255
         Rox_Sshort r2 = (image_inp_data[cy][cx+1] & 0x000000ff) - r1;//maxval = 511
         Rox_Sshort r3 = (image_inp_data[cy+1][cx] & 0x000000ff) - r1;//maxval = 511
         Rox_Sshort r4 = r1 + (image_inp_data[cy+1][cx+1] & 0x000000ff) - (image_inp_data[cy+1][cx] & 0x000000ff) - (image_inp_data[cy][cx+1] & 0x000000ff);//maxval = 2048

         //255 + 511*16 + 511*16+ 2048*16
         val = ((((r1 << 4) + r2*dx + r3*dy) << 4) + r4*dx*dy);
         Rox_Uint r = (val >> 8) & 255;
   
         // green
         Rox_Sshort g1 = (image_inp_data[cy][cx] >> 8 & 0x000000ff);//maxval = 255
         Rox_Sshort g2 = (image_inp_data[cy][cx+1] >> 8 & 0x000000ff) - g1;//maxval = 511
         Rox_Sshort g3 = (image_inp_data[cy+1][cx] >> 8 & 0x000000ff) - g1;//maxval = 511
         Rox_Sshort g4 = g1 + (image_inp_data[cy+1][cx+1] >> 8 & 0x000000ff) - (image_inp_data[cy+1][cx] >> 8 & 0x000000ff) - (image_inp_data[cy][cx+1] >> 8 & 0x000000ff);//maxval = 2048

         //255 + 511*16 + 511*16+ 2048*16
         val = ((((g1 << 4) + g2*dx + g3*dy) << 4) + g4*dx*dy);
         Rox_Uint g = (val >> 8) & 255;
   
         // blue
         Rox_Sshort b1 = (image_inp_data[cy][cx] >> 16 & 0x000000ff);//maxval = 255
         Rox_Sshort b2 = (image_inp_data[cy][cx+1] >> 16 & 0x000000ff) - b1;//maxval = 511
         Rox_Sshort b3 = (image_inp_data[cy+1][cx] >> 16 & 0x000000ff) - b1;//maxval = 511
         Rox_Sshort b4 = b1 + (image_inp_data[cy+1][cx+1] >> 16 & 0x000000ff) - (image_inp_data[cy+1][cx] >> 16 & 0x000000ff) - (image_inp_data[cy][cx+1] >> 16 & 0x000000ff);//maxval = 2048

         //255 + 511*16 + 511*16+ 2048*16
         val = ((((b1 << 4) + b2*dx + b3*dy) << 4) + b4*dx*dy);
         Rox_Uint b = (val >> 8) & 255;
   
         // build output rgba
         image_out_data[i][j] = ((255 << 24) + (b << 16) + (g << 8) + r);
   
      }
   }
   
function_terminate:
   return error;
}