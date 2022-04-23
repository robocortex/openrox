//============================================================================
//
//    OPENROX   : File band.c
//
//    Contents  : Implementation of band module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "band.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_band ( Rox_Array2D_Uint res, const Rox_Array2D_Uint one, const Rox_Array2D_Uint two )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D) res, (Rox_Array2D) one);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_match_size((Rox_Array2D) one, (Rox_Array2D) two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dres = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** done = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &done, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dtwo = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dtwo, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         dres[i][j] = done[i][j] & dtwo[i][j];
      }
   }

function_terminate:
   return error;
}
