//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_float_to_float.c
//
//    Contents  : Implementation of remap_bilinear_nomask_float_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_nomask_float_to_float.h"

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_bilinear_nomask_float_to_float (
   Rox_Array2D_Float image_out, 
   const Rox_Array2D_Float image_inp, 
   const Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !image_out )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image_inp || !grid ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols_out = 0, rows_out = 0;
   error  = rox_array2d_float_get_size ( &rows_out, &cols_out, image_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_inp = 0, rows_inp = 0;
   error  = rox_array2d_float_get_size ( &rows_inp, &cols_inp, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size ( grid, rows_out, cols_out ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** out_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &out_data, image_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** inp_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &inp_data, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows_out; i++ )
   {
      for ( Rox_Sint j = 0; j < cols_out; j++ )
      {
         Rox_Float I00 = 0.0f;
         Rox_Float I01 = 0.0f;
         Rox_Float I10 = 0.0f;
         Rox_Float I11 = 0.0f;

         // Get the float coordinates
         Rox_Float uf = grid_u_data[i][j];
         Rox_Float vf = grid_v_data[i][j];

         out_data[i][j] = 0.0f;

         // Get the integer coordinates
         Rox_Sint ui = (Rox_Sint) uf;
         Rox_Sint vi = (Rox_Sint) vf;

         if ((ui < 0) || (vi < 0)) continue;
         if ((ui > cols_inp - 1) || (vi > rows_inp - 1)) continue;
         // Compute the residuals
         
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
            out_data[i][j] = I00 * (1 - dv) + dv * I10;
            continue;
         }

         if ((( uf >= 0.0 ) && ( vf < 0.0 )) || (( uf < cols_inp-1 ) && ( vf >= rows_inp-1 )))
         {
            I00 = inp_data[vi][ui];
            I01 = inp_data[vi][ui+1];
            out_data[i][j] = I00 * (1 - du) + du * I01;
            continue;
         }
         
         I00 = inp_data[vi][ui];
         I01 = inp_data[vi][ui+1];
         I10 = inp_data[vi+1][ui];
         I11 = inp_data[vi+1][ui+1];
#else
         I00 = inp_data[vi][ui];
         I01 = 0.0f;
         I10 = 0.0f;
         I11 = 0.0f;

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

         out_data[i][j] = b1 + b2 * du + b3 * dv + b4 * du * dv;
      }
   }

function_terminate:
   return error;
}
