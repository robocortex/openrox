//============================================================================
//
//    OPENROX   : File set_data.h
//
//    Contents  : API of set_data module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "set_data.h"
#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_set_data ( Rox_Array2D_Uint dest, Rox_Uint * source, Rox_Sint bytesPerRow)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!dest || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for(Rox_Sint i = 0; i < rows; i++)
   {
      memcpy(dd[i], source + i * bytesPerRow, cols * sizeof(*source));
   }

function_terminate:
   return error;
}
