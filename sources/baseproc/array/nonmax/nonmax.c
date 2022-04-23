//==============================================================================
//
//    OPENROX   : File nonmax.c
//
//    Contents  : Implementation of nonmax module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "nonmax.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_nonmax(Rox_Array2D_Float output, Rox_Array2D_Float input, Rox_Uint radius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ismax;
   Rox_Float cmpval;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(output, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **dres = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dres, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **din = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &din, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Sint top, bottom;

      top = i - radius;
      bottom = i + radius;
      if (top < 0) top = 0;
      if (bottom > rows - 1) bottom = rows - 1;

      for ( Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Sint left, right;

         left = j - radius;
         right = j + radius;
         if (left < 0) left = 0;
         if (right > cols - 1) right = cols - 1;

         cmpval = din[i][j];
         ismax = 1;
         for ( Rox_Sint k = top; k <= bottom && ismax; k++)
         {
            for ( Rox_Sint l = left; l <= right && ismax; l++)
            {
               if (k == i && j == l) continue;

               if (cmpval <= din[k][l])
               {
                  ismax = 0;
                  cmpval = 0;
               }
            }
         }

         dres[i][j] = cmpval;
      }
   }

function_terminate:
   return error;
}
