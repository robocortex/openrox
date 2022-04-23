//==============================================================================
//
//    OPENROX   : File transpose.c
//
//    Contents  : Implementation of transpose module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "transpose.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_transpose ( Rox_Array2D_Double out, Rox_Array2D_Double inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!out || !inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols_out = 0, rows_out = 0; 
   error = rox_array2d_double_get_size(&rows_out, &cols_out, out);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint cols_inp = 0, rows_inp = 0; 
   error = rox_array2d_double_get_size(&rows_inp, &cols_inp, inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows_out != cols_inp) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (cols_out != rows_inp) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** data_out = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data_out, out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data_inp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data_inp, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows_out; i++)
   {
      for (Rox_Sint j = 0; j < cols_out; j++)
      {
         data_out[i][j] = data_inp[j][i];
      }
   }

function_terminate:
   return error;
}