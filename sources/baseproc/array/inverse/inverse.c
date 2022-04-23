//==============================================================================
//
//    OPENROX   : File inverse.c
//
//    Contents  : Implementation of inverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inverse.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_inverse ( Rox_Array2D_Float Mi, const Rox_Array2D_Float M )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_float_match_size(Mi, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, M ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set Li matrix to 0
   error = rox_array2d_float_fillval ( Mi, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** M_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &M_data, M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Mi_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Mi_data, Mi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint r = 0; r < rows; r++)
   {
      for (Rox_Sint c = 0; c < cols; c++)
      {
         if ( M_data[r][c] != 0 )
         {
            Mi_data[r][c] = 1/M_data[r][c];
         }
      }
   }

function_terminate:
   return error;
}
