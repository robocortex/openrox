//==============================================================================
//
//    OPENROX   : File remap_bilinear_uchar_to_float.c
//
//    Contents  : Implementation of remap_bilinear_uchar_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_uchar_to_float.h"

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

int rox_ansi_remap_bilinear_uchar_to_float (
   float ** image_out_data,
   unsigned int ** imask_out_data,
   unsigned int ** imask_out_ini_data,
   int rows_out,
   int cols_out,
   unsigned char ** image_inp_data,
   unsigned int ** imask_inp_data,
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
)
{
   int error = 0;

   for ( int i = 0; i < rows_out; i++ )
   {
      for ( int j = 0; j < cols_out; j++ )
      {
         // Get the float coordinates
         float uf = grid_u_data[i][j];
         float vf = grid_v_data[i][j];

         image_out_data[i][j] = 0.0f;
         imask_out_data[i][j] = 0;

         if ( !imask_inp_data[i][j] ) continue;
         
         if ((uf < 0.0f) || (vf < 0.0f)) continue;
         if ((uf > cols_inp - 1.0f) || (vf > rows_inp - 1.0f)) continue;

         // Get the integer coordinates
         int ui = (int) (uf);
         int vi = (int) (vf);

         float I00 = image_inp_data[vi][ui];
         float I01 = 0.0f;
         float I10 = 0.0f;
         float I11 = 0.0f;

         if ( !imask_inp_data[vi][ui]         ) continue;

         // Added tests for borders special case
         if ( ui != cols_inp - 1 ) 
         {
            if ( !imask_inp_data[vi][ui + 1]     ) continue;
            I01 = image_inp_data[vi][ui + 1];
         }

         if ( vi != rows_inp - 1 ) 
         {
            if ( !imask_inp_data[vi + 1][ui]     ) continue;
            I10 = image_inp_data[vi + 1][ui];
         }

         if ((ui != cols_inp - 1) && (vi != rows_inp - 1))
         {
            if ( !imask_inp_data[vi + 1][ui + 1] ) continue;
            I11 = image_inp_data[vi + 1][ui + 1];
         }

         // Compute the residuals
         float du = uf - ui;
         float dv = vf - vi;

         // Bilinear interpolation
         float b1 = I00;
         float b2 = I01 - b1;
         float b3 = I10 - b1;
         float b4 = b1 + I11 - I10 - I01;

         image_out_data[i][j] = b1 + b2 * du + b3 * dv + b4 * du * dv;
         imask_out_data[i][j] = ~0;
      }
   }

   return error;
}
