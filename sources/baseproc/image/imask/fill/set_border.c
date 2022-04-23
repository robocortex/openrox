//============================================================================
//
//    OPENROX   : File set_border.h
//
//    Contents  : API of set_border module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "set_border.h"
#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_set_border(Rox_Array2D_Uint dest, Rox_Sint size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (size == 0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (size >= rows || size >= cols) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint **data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Mask first and last column 
   for(Rox_Sint v = 0; v < rows; v++)
   {
      for(Rox_Sint u = 0; u < size; u++)
      {
         data[v][u]        = 0;
         data[v][cols-u-1] = 0;
      }
   }

   // Mask first and last row 
   for(Rox_Sint v = 0; v < size; v++)
   {
      memset(data[v], 0, cols * sizeof(Rox_Uint));
      memset(data[rows-v-1], 0, cols * sizeof(Rox_Uint));
   }

function_terminate:
   return error;
}
