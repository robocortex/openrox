//==============================================================================
//
//    OPENROX   : File depthfromdisparity.c
//
//    Contents  : Implementation of depthfromdisparity module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "depthfromdisparity.h"
#include <float.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_depth_from_disparity (
   Rox_Array2D_Float depth, 
   Rox_Array2D_Uint output_mask, 
   const Rox_Array2D_Float disparity, 
   const Rox_Double fu_left, 
   const Rox_Double cu_left, 
   const Rox_Double fu_right, 
   const Rox_Double cu_right, 
   const Rox_Double baseline
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!depth || !output_mask || !disparity) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (baseline > DBL_EPSILON) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, depth); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(output_mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_check_size(disparity, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dz = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dz, depth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, disparity);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint  ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, output_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The inversion should be done outside of the function and passed to the function
   Rox_Double ifu_left = 1.0 / fu_left;
   Rox_Double ifu_right = 1.0 / fu_right;
   Rox_Double icu_left = - cu_left * ifu_left;
   Rox_Double icu_right = - cu_right * ifu_right;

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Float disp = dd[i][j];

         dz[i][j] = 0;
         dm[i][j] = 0;

         if (disp < 1e-5) continue;

         Rox_Float xl = (Rox_Float) ( ifu_left * ((Rox_Double)j) + icu_left );
         Rox_Float xr = (Rox_Float) ( ifu_right * (- disp + (Rox_Double)j) + icu_right );

         dz[i][j] = ((Rox_Float) baseline) / (xr - xl);
         dm[i][j] = ~0;
      }
   }

function_terminate:
   return error;
}
