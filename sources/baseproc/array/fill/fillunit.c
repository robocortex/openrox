//==============================================================================
//
//    OPENROX   : File fillunit.c
//
//    Contents  : Implementation of fillunit module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fillunit.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_fillunit(Rox_Array2D_Double dest)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (i == j) data[i][j] = 1.0;
         else data[i][j] = 0.0;
      }
   }

function_terminate:
   return error;
}
