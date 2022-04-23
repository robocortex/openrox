//==============================================================================
//
//    OPENROX   : File bnot.c
//
//    Contents  : Implementation of bnot module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bnot.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_bnot(Rox_Array2D_Uint dest, Rox_Array2D_Uint source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   error = rox_array2d_uint_match_size ( dest, source ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL; 
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         dd[i][j] = ~ds[i][j];
      }
   }

function_terminate:
   return error;
}
