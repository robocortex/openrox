//==============================================================================
//
//    OPENROX   : File lotinverse.c
//
//    Contents  : Implementation of lotinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "lotinverse.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_lotinverse ( Rox_Array2D_Double Li, const Rox_Array2D_Double L )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_match_size(Li, L);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, L ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols != rows || cols < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set Li matrix to 0
   error = rox_array2d_double_fillval ( Li, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** L_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &L_data, L );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Li_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Li_data, Li );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < cols; i++)
   {
      Li_data[i][i] = 1/L_data[i][i];
      for (Rox_Sint j = 0; j < i; j++)
      {
         Rox_Double s = 0.0;
         for (Rox_Sint k = j; k <= i; k++)
         {
            s += L_data[i][k] * Li_data[k][j];
         }
         Li_data[i][j] = -s*Li_data[i][i];
      }
   }

function_terminate:
   return error;
}
