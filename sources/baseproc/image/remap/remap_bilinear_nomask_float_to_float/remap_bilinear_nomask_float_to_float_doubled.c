//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_float_to_float_doubled.c
//
//    Contents  : Implementation of remap doubled module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_nomask_float_to_float_doubled.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode remap_bilinear_nomask_float_to_float_doubled (
   Rox_Array2D_Float dest,
   const Rox_Array2D_Float source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint dcols = cols * 2;
   Rox_Sint drows = rows * 2;

   error = rox_array2d_float_check_size(dest, drows, dcols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < drows; i++)
   {
      Rox_Double di = ((Rox_Double) i) / 2.0;
      Rox_Sint dr = (Rox_Sint) di;
      Rox_Double dy = di - (Rox_Double) dr;

      for (Rox_Sint j = 0; j < dcols; j++)
      {
         Rox_Double a, b, c, d;

         Rox_Double dj = ((Rox_Double) j) / 2.0;
         Rox_Sint dc = (Rox_Sint) dj;
         Rox_Double dx = dj - (Rox_Double) dc;

         if (dr == rows - 1 && dc == cols - 1)
         {
            a = ds[dr][dc];
            b = a;
            c = b;
            d = c;
         }
         else if (dr == rows - 1)
         {
            a = ds[dr][dc];
            b = ds[dr][dc + 1];
            c = b;
            d = c;
         }
         else if (dc == cols - 1)
         {
            a = ds[dr][dc];
            b = a;
            c = ds[dr + 1][dc];
            d = c;
         }
         else
         {
            a = ds[dr][dc];
            b = ds[dr][dc + 1];
            c = ds[dr + 1][dc];
            d = ds[dr + 1][dc + 1];
         }

         Rox_Double b1 = a;
         Rox_Double b2 = b - b1;
         Rox_Double b3 = c - b1;
         Rox_Double b4 = b1 + d - c - b;

         dd[i][j] = (Rox_Float) (b1 + b2 * dx + b3 * dy + b4 * dx * dy);
      }
   }

function_terminate:
   return error;
}
