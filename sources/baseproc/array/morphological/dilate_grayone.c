//==============================================================================
//
//    OPENROX   : File dilate_grayone.c
//
//    Contents  : Implementation of dilate_grayone module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#
#include "dilate_grayone.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_dilate_grayone(Rox_Array2D_Float res, Rox_Array2D_Float input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(input, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dres = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** din = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &din, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Sint previ = i - 1;
      if (previ < 0) previ = i;
      Rox_Sint nexti = i + 1;
      if (nexti >= rows - 1) nexti = i;

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Sint prevj = j - 1;
         if (prevj < 0) prevj = j;
         Rox_Sint nextj = j + 1;
         if (nextj >= cols - 1) nextj = j;

         Rox_Float maxval = din[i][j];
         maxval = ROX_MAX(maxval, din[previ][prevj]);
         maxval = ROX_MAX(maxval, din[previ][j]);
         maxval = ROX_MAX(maxval, din[previ][nextj]);
         maxval = ROX_MAX(maxval, din[nexti][prevj]);
         maxval = ROX_MAX(maxval, din[nexti][j]);
         maxval = ROX_MAX(maxval, din[nexti][nextj]);
         maxval = ROX_MAX(maxval, din[i][prevj]);
         maxval = ROX_MAX(maxval, din[i][nextj]);

         dres[i][j] = maxval;
      }
   }

function_terminate:
   return error;
}

