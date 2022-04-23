//=============================================================================
//
//    OPENROX   : File basic_convolve.c
//
//    Contents  : Implementation of basic_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#include "basic_convolve.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_convolve_with_kernel (
   Rox_Array2D_Float dest, 
   const Rox_Array2D_Float source, 
   const Rox_Array2D_Float kernel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!kernel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   
   error = rox_array2d_float_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(source, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint kcols = 0, krows = 0;
   error = rox_array2d_float_get_size(&krows, &kcols, kernel); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hkcols = kcols / 2;
   Rox_Sint hkrows = krows / 2;

   if (kcols % 2 == 0) 
   { error = ROX_ERROR_VALUE_NOT_ODD; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (krows % 2 == 0) 
   { error = ROX_ERROR_VALUE_NOT_ODD; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float ** dsource = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dsource, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ddest = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&ddest, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dkernel = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dkernel, kernel);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Sint top = ROX_MAX(0, i - hkrows);
      Rox_Sint bottom = ROX_MIN(rows - 1, i + hkrows);

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Sint left = ROX_MAX(0, j - hkcols);
         Rox_Sint right = ROX_MIN(cols - 1, j + hkcols);

         Rox_Double sum = 0.0;
         Rox_Double normer = 0.0;

         for (Rox_Sint k = top; k <= bottom; k++)
         {
            for (Rox_Sint l = left; l <= right; l++)
            {
               normer += dkernel[i - k + hkrows][l - j + hkcols];
               sum += dsource[k][l] * dkernel[i - k + hkrows][l - j + hkcols];
            }
         }

         ddest[i][j] = (Rox_Float) (sum / normer);
      }
   }

function_terminate:
   return error;
}
