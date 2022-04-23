//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_uchar_to_uchar.c
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
	
   if ( !output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !input || !grid ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols_out = 0, rows_out = 0;
   error  = rox_array2d_uchar_get_size ( &rows_out, &cols_out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_inp = 0, rows_inp = 0;
   error  = rox_array2d_uchar_get_size ( &rows_inp, &cols_inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size ( grid, rows_out, cols_out ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &out_data, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** inp_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &inp_data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows_out; i++)
   {
      for (Rox_Sint j = 0; j < cols_out; j++)
      {
         Rox_Float I00 = 0.0f;
         Rox_Float I01 = 0.0f;
         Rox_Float I10 = 0.0f;
         Rox_Float I11 = 0.0f;
         Rox_Float out_val = 0.0f;

         // Get the float coordinates
         Rox_Float uf = grid_u_data[i][j];
         Rox_Float vf = grid_v_data[i][j];

         out_data[i][j] = 0;

         // Get the integer coordinates
         Rox_Sint ui = (Rox_Sint) uf;
         Rox_Sint vi = (Rox_Sint) vf;

         if ( (ui < 0) || (vi < 0) ) continue;
         if ( (ui > cols_inp - 1) || (vi > rows_inp - 1) ) continue;

         Rox_Float du = uf - ui;
         Rox_Float dv = vf - vi;

#ifndef EXTRAPOLATION_ON_BORDERS
         if ((( uf < 0.0 ) && ( vf < 0.0 )) || (( uf >= cols_inp-1 ) && ( vf >= rows_inp-1 )) || (( uf >= cols_inp-1 ) && ( vf < 0 )) || (( uf < 0 ) && ( vf >= rows_inp-1 )))
         {
            out_data[i][j] = inp_data[vi][ui];
            continue;
         }
         
         if ((( uf < 0.0 ) && ( vf >= 0.0 )) || (( uf >= cols_inp-1 ) && ( vf < rows_inp-1 )))
         {
            I00 = inp_data[vi][ui];
            I10 = inp_data[vi+1][ui];
            out_val = I00 * (1 - dv) + dv * I10;
            out_data[i][j] = (Rox_Uchar) (out_val + 0.5f);
            continue;
         }

         if ((( uf >= 0.0 ) && ( vf < 0.0 )) || (( uf < cols_inp-1 ) && ( vf >= rows_inp-1 )))
         {
            I00 = inp_data[vi][ui];
            I01 = inp_data[vi][ui+1];
            out_val = I00 * (1 - du) + du * I01;
            out_data[i][j] = (Rox_Uchar) (out_val + 0.5f);
            continue;
         }
         
         I00 = inp_data[vi][ui];
         I01 = inp_data[vi][ui+1];
         I10 = inp_data[vi+1][ui];
         I11 = inp_data[vi+1][ui+1];
#else
         I00 = inp_data[vi][ui];
         I01 = 0.0;
         I10 = 0.0;
         I11 = 0.0;

         // Added test for borders special case
         if ( ui != cols_inp - 1 ) 
         {
            I01 = inp_data[vi][ui + 1];
         }

         if ( vi != rows_inp - 1 ) 
         {
            I10 = inp_data[vi + 1][ui];
         }

         if ((ui != cols_inp - 1) && (vi != rows_inp - 1))
         {
            I11 = inp_data[vi + 1][ui + 1];
         }
#endif
         // Bilinear interpolation
         Rox_Float b1 = I00;
         Rox_Float b2 = I01 - b1;
         Rox_Float b3 = I10 - b1;
         Rox_Float b4 = b1 + I11 - I10 - I01;

         out_val = b1 + b2 * du + b3 * dv + b4 * du * dv;

         out_data[i][j] = (Rox_Uchar) (out_val + 0.5f);
        }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_remap_bilinear_nomask_uchar_to_uchar_fixed (
   Rox_Image output, 
   const Rox_Image input, 
   const Rox_Array2D_Point2D_Sshort grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input || !grid) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols_out = 0, rows_out = 0;
   error  = rox_array2d_uchar_get_size(&rows_out, &cols_out, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_inp = 0, rows_inp = 0;
   error  = rox_array2d_uchar_get_size(&rows_inp, &cols_inp, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_check_size(grid, rows_out, cols_out); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out = NULL; 
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &out, output);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar ** inp = NULL; 
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &inp, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort * mxy = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &mxy, grid);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows_out; i++)
   {
      for (Rox_Sint j = 0; j < cols_out; j++)
      {
         out[i][j] = 255;

         Rox_Sint cx = mxy[i][j].u >> 4;
         Rox_Sint cy = mxy[i][j].v >> 4;

         Rox_Sshort dx = mxy[i][j].u & 15;
         Rox_Sshort dy = mxy[i][j].v & 15;

         if (cx < 0 || cy < 0) continue;
         if (cx >= cols_inp - 1 || cy >= rows_inp - 1) continue;

         Rox_Sshort b1 = inp[cy][cx]; //maxval = 255
         Rox_Sshort b2 = inp[cy][cx+1] - b1; //maxval = 511
         Rox_Sshort b3 = inp[cy+1][cx] - b1; //maxval = 511
         Rox_Sshort b4 = b1 + inp[cy+1][cx+1] - inp[cy+1][cx] - inp[cy][cx+1]; //maxval = 2048

         //255 + 511*16 + 511*16+ 2048*16
         Rox_Sint val = ((((b1 << 4) + b2*dx + b3*dy) << 4) + b4*dx*dy);
         out[i][j] = (val >> 8) & 255;
      }
   }

function_terminate:
   return error;
}