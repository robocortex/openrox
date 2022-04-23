//==============================================================================
//
//    OPENROX   : File scale.c
//
//    Contents  : Implementation of scale module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "scale.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_scale(Rox_Array2D_Double out, Rox_Array2D_Double inp, Rox_Double scale)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_match_size(out, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **out_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &out_data, out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **inp_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &inp_data, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         out_data[i][j] =  inp_data[i][j] * scale;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_scale_inplace(Rox_Array2D_Double inpout, Rox_Double scale)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, inpout); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** inpout_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &inpout_data, inpout);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         inpout_data[i][j] =  inpout_data[i][j] * scale;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_scale (
   Rox_Array2D_Float out, 
   const Rox_Array2D_Float inp, 
   const Rox_Float scale
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_float_match_size(out, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **out_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &out_data, out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **inp_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &inp_data, inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         out_data[i][j] = inp_data[i][j] * scale;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_scale_inplace ( Rox_Array2D_Float inpout, const Rox_Float scale )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, inpout ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** inpout_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &inpout_data, inpout );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         inpout_data[i][j] =  inpout_data[i][j] * scale;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_scale_col(Rox_Array2D_Double out, Rox_Array2D_Double inp, Rox_Double scale, Rox_Sint col)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_match_size(out, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (col < 0 || col >= cols) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double **out_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&out_data, out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **inp_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&inp_data, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      out_data[i][col] =  inp_data[i][col] * scale;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_scale_row(Rox_Array2D_Double out, Rox_Array2D_Double inp, Rox_Double scale, Rox_Sint row)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_match_size(out, inp); 
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if(row < 0 || row >= rows) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double **out_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&out_data, out);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double **inp_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&inp_data, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint j = 0; j < cols; j++)
   {
      out_data[row][j] =  inp_data[row][j] * scale;
   }
 
function_terminate:
   return error;
}
