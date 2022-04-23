//==============================================================================
//
//    OPENROX   : File remap_bilinear_uchar_to_uchar.c
//
//    Contents  : Implementation of remap_bilinear_uchar_to_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_uchar_to_uchar.h"

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_bilinear_uchar_to_uchar (
   Rox_Image output, 
   Rox_Array2D_Uint mask_out, 
   const Rox_Array2D_Uint mask_out_ini, 
   const Rox_Image input, 
   const Rox_Array2D_Uint mask_inp, 
   const Rox_MeshGrid2D_Float grid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !output || !mask_out )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !mask_out_ini || !mask_inp || !input || !grid ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols_out = 0, rows_out = 0;
   error = rox_array2d_uchar_get_size ( &rows_out, &cols_out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_inp = 0, rows_inp = 0;
   error = rox_array2d_uchar_get_size ( &rows_inp, &cols_inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_check_size ( mask_out_ini, rows_out, cols_out );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_check_size ( mask_out, rows_out, cols_out ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_check_size ( mask_inp, rows_inp, cols_inp );     
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size ( grid, rows_out, cols_out );       
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &out_data, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint  ** in_mask = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &in_mask, mask_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint  ** mask_inp_data  = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_inp_data, mask_out_ini );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint  ** mask_out_data  = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_out_data, mask_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** inp_data         = NULL;
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
         // Get the float coordinates
         Rox_Float uf = grid_u_data[i][j];
         Rox_Float vf = grid_v_data[i][j];

         out_data[i][j] = 0;
         mask_out_data[i][j] = 0;

         if ( !mask_inp_data[i][j] ) continue;

         if ((uf < 0.0f) || (vf < 0.0f)) continue;
         if ((uf > cols_inp - 1.0f) || (vf > rows_inp - 1.0f)) continue;

         Rox_Sint ui = (Rox_Sint) (uf);
         Rox_Sint vi = (Rox_Sint) (vf);

         Rox_Float I00 = (Rox_Float) inp_data[vi][ui];
         Rox_Float I01 = 0.0f;
         Rox_Float I10 = 0.0f;
         Rox_Float I11 = 0.0f;

         if ( !in_mask[vi][ui] ) continue;

         // Added tests for borders special case
         if ( ui != cols_inp - 1 ) 
         {
            if ( !in_mask[vi][ui + 1] ) continue;
            I01 = (Rox_Float) inp_data[vi][ui + 1];
         }

         if ( vi != rows_inp - 1 ) 
         {
            if ( !in_mask[vi + 1][ui] ) continue;
            I10 = (Rox_Float) inp_data[vi + 1][ui];
         }

         if ((ui != cols_inp - 1) && (vi != rows_inp - 1))
         {
            if ( !in_mask[vi + 1][ui + 1] ) continue;
            I11 = (Rox_Float)  inp_data[vi + 1][ui + 1];
         }

         // Compute the residuals
         Rox_Float du = uf - ui;
         Rox_Float dv = vf - vi;

         // Bilinear interpolation
         Rox_Float b1 = I00;
         Rox_Float b2 = I01 - b1;
         Rox_Float b3 = I10 - b1;
         Rox_Float b4 = b1 + I11 - I10 - I01;

         Rox_Float out_val = b1 + b2 * du + b3 * dv + b4 * du * dv;

         out_data[i][j] = (Rox_Uchar) (out_val + 0.5f);
         mask_out_data[i][j] = ~0;
      }
   }

function_terminate:
   return error;
}
