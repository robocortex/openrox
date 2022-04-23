//==============================================================================
//
//    OPENROX   : File flipud.c
//
//    Contents  : Implementation of flipud module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "flipud.h"

#include <string.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_copy_flip_ud(Rox_Array2D_Double output, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!input || !output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_double_get_stride(&stride, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(input, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** rows_out = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &rows_out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** rows_inp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &rows_inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint last = rows - 1;

   Rox_Double *swap = (Rox_Double *) rox_memory_allocate(sizeof(Rox_Double), stride);
   
   for (Rox_Sint i = 0; i <= rows / 2; i++)
   {
      memcpy(swap, rows_inp[i], stride);
      memcpy(rows_out[i], rows_inp[last - i], stride);
      memcpy(rows_out[last - i], swap, stride);
   }

   rox_memory_delete(swap);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_copy_flip_ud(Rox_Array2D_Uchar output, Rox_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input || !output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride(&stride, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(input, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** rows_out = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &rows_out, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** rows_inp = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &rows_inp, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint last = rows - 1;

   Rox_Uchar * swap = (Rox_Uchar *) rox_memory_allocate(sizeof(Rox_Uchar), stride);
   
   for (Rox_Sint i = 0; i <= rows / 2; i++)
   {
      memcpy(swap, rows_inp[i], stride);
      memcpy(rows_out[i], rows_inp[last - i], stride);
      memcpy(rows_out[last - i], swap, stride);
   }

   rox_memory_delete(swap);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_flip_ud(Rox_Array2D_Uchar inout)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!inout) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_uchar_get_rows(&rows, inout); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dinout = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dinout, inout);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i <= rows / 2; i++)
   {
      Rox_Sint last = rows - 1;
      Rox_Uchar *swap = dinout[i]; // In case of inplace operation
      dinout[i] = dinout[last - i];
      dinout[last - i] = swap;    
   }
   
function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_uint_flip_ud(Rox_Array2D_Uint inout)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!inout)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint height;
   error = rox_array2d_uint_get_rows(&height, inout);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dinout = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dinout, inout );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i <= height / 2; i++)
   {
      Rox_Uint * swap = NULL;
      Rox_Sint last = height - 1;
      swap = dinout[i]; // In case of inplace operation
      dinout[i] = dinout[last - i];
      dinout[last - i] = swap;
   }

function_terminate:
   return error;
}
