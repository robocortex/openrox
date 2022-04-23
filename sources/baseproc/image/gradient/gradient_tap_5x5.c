//==============================================================================
//
//    OPENROX   : File gradient_tap_5x5.c
//
//    Contents  : Implementation of gradientsobel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "gradient_tap_5x5.h"
#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_gradient5(Rox_Array2D_Float gx, Rox_Array2D_Float gy, Rox_Array2D_Float source, Rox_Array2D_Float buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!gx || !gy || !buffer || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, gx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint max = cols; if (rows > max) max = rows;

   error = rox_array2d_float_check_size(gy, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(source, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_buffer = 0;
   error = rox_array2d_float_get_cols(&cols_buffer, buffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols_buffer < (Rox_Sint) max)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float ** dsrc = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dsrc, source);

   Rox_Float ** dgx = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dgx, gx);

   Rox_Float ** dgy = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dgy, gy);

   Rox_Float * dbuffer = NULL;
   error = rox_array2d_float_get_data_pointer ( &dbuffer, buffer );

   rox_array2d_float_fillval(gx, 0);
   rox_array2d_float_fillval(gy, 0);

   // Compute horizontal gradient
   for ( Rox_Sint i = 2; i < rows - 2; i++)
   {
      // Compute interpolant
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         dbuffer[j] = (Rox_Float)(dsrc[i-2][j] * 0.037659 + dsrc[i-1][j] * 0.249153 + dsrc[i][j] * 0.426375 + dsrc[i+1][j] * 0.249153 + dsrc[i+2][j] * 0.037659);
      }

      // Compute horizontal gradient
      for ( Rox_Sint j = 2; j < cols - 2; j++)
      {
         dgx[i][j] = (Rox_Float)(dbuffer[j-2] * 0.109604 + dbuffer[j-1] * 0.276691 + dbuffer[j+1] * -0.276691 + dbuffer[j+2] * -0.109604);
      }
   }

   // Compute horizontal gradient
   for ( Rox_Sint i = 2; i < cols - 2; i++)
   {
      // Compute interpolant
      for ( Rox_Sint j = 0; j < rows; j++)
      {
         dbuffer[j] = (Rox_Float)(dsrc[j][i-2] * 0.037659 + dsrc[j][i-1] * 0.249153 + dsrc[j][i] * 0.426375 + dsrc[j][i+1] * 0.249153 + dsrc[j][i+2] * 0.037659);
      }

      // Compute horizontal gradient
      for ( Rox_Sint j = 2; j < rows - 2; j++)
      {
         dgy[j][i] = (Rox_Float) (dbuffer[j-2] * 0.109604 + dbuffer[j-1] * 0.276691 + dbuffer[j+1] * -0.276691 + dbuffer[j+2] * -0.109604);
      }
   }

function_terminate:
   return error;
}