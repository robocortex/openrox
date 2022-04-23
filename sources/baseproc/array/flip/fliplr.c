//==============================================================================
//
//    OPENROX   : File flip_lr.c
//
//    Contents  : Implementation of flip_lr module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fliplr.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_copy_flip_lr(Rox_Array2D_Double output, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input || !output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(input, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **din = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &din, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint last = cols - 1;

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j <= cols / 2; j++)
      {
         Rox_Double swap = din[i][j]; // In case of inplace operation
         dout[i][j] = din[i][last - j];
         dout[i][last - j] = swap;
      }
   }

function_terminate:
   return error;
}
