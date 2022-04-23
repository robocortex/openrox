//==============================================================================
//
//    OPENROX   : File linsys_texture_rxry_light_affine.c
//
//    Contents  : Implementation of linsys_texture_rxry_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_rxry_light_affine.h"

#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode linsys_texture_rxry_light_affine (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   Rox_Array2D_Float diffs, 
   Rox_Array2D_Float gradient_x, 
   Rox_Array2D_Float gradient_y, 
   Rox_Imask input_mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !diffs || !gradient_x || !gradient_y || !input_mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, diffs); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(input_mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gradient_x, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gradient_y, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(diffs, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dLtL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dLtL, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** djtf = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &djtf, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd= NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, diffs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dgx = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgx, gradient_x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dgy = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgy, gradient_y);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, input_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint count = 0;
   for (Rox_Sint i = 1; i < rows - 1; i++)
   {
      for (Rox_Sint j = 1; j < cols - 1; j++)
      {
         Rox_Double J[3];

         if (!dm[i][j]) continue;

         Rox_Double d = dd[i][j];
         Rox_Double Iu = dgx[i][j];
         Rox_Double Iv = dgy[i][j];

         J[0] = Iu;
         J[1] = Iv;
         J[2] = 1;

         for (Rox_Sint k = 0; k < 3; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               dLtL[k][l] += J[k]*J[l];
            }

            djtf[k][0] += J[k] * d;
         }

         count++;
      }
   }

   for (Rox_Sint k = 0; k < 3; k++)
   {
      for (Rox_Sint l = 0; l <= k; l++)
      {
         dLtL[l][k] = dLtL[k][l];
      }
   }

function_terminate:
   return error;
}
