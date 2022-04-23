//==============================================================================
//
//    OPENROX   : File gausspivoting.c
//
//    Contents  : Implementation of gausspivoting module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "gausspivoting.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_gauss_pivoting(Rox_Array2D_Double output, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint r, c, row, col, pivot;
   Rox_Double maxabs,swap, scale, sub;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(output, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(output, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   r = 0;
   // Loop over columns
   for (c = 0; c < (Rox_Uint) cols; c++)
   {
      // Find the biggest absolute value in this column under the current row
      maxabs = 0;
      pivot = 0;
      for (row = r; row < (Rox_Uint) rows; row++)
      {
         if (fabs(dout[row][c]) > maxabs)
         {
            maxabs = fabs(dout[row][c]);
            pivot = row;
         }
      }

      // This maximum cell is the pivot
      if (maxabs == 0.0)
      {
         // If pivot is null, nullify column
         for (row = r; row < (Rox_Uint) rows; row++)
         {
            dout[row][c] = 0;
         }
      }
      else
      {
         // Swap pivot row and current row
         for (col = 0; col < (Rox_Uint) cols; col++)
         {
            swap = dout[pivot][col];
            dout[pivot][col] = dout[r][col];
            dout[r][col] = swap;
         }

         // Scale row such that the diagonal is 1
         scale = 1.0 / dout[r][c];
         for (col = 0; col < (Rox_Uint) cols; col++)
         {
            dout[r][col] = dout[r][col] * scale;
         }

         // Update each rows such that the current column equals 0 (except on the current row)
         for (row = 0; row < (Rox_Uint) rows; row++)
         {
            if (row == r) continue;

            sub = dout[row][c];
            for (col = 0; col < (Rox_Uint) cols; col++)
            {
               dout[row][col] = dout[row][col] - sub  * dout[r][col];
            }
         }

         // Stop if we met the bottom border
         r++;
         if (r == rows) break;
      }
   }

function_terminate:
   return error;
}
