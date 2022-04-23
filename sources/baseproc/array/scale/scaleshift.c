//==============================================================================
//
//    OPENROX   : File scaleshift.c
//
//    Contents  : Implementation of scaleshift module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "scaleshift.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_scaleshift(Rox_Array2D_Float res, Rox_Array2D_Float one, Rox_Float scale, Rox_Float shift)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !one) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(one, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dres = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** done = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &done, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dres[i][j] =  done[i][j] * scale + shift;
      }
   }

function_terminate:
   return error;
}
