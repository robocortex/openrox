//==============================================================================
//
//    OPENROX   : File centered_error.c
//
//    Contents  : Implementation of centered_error module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "centered_error.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_centered_error ( 
   Rox_Double * median, 
   Rox_Double * sum_square, 
   Rox_Uint * count_valid, 
   Rox_Array2D_Float res, 
   Rox_Array2D_Uint mask, 
   Rox_Array2D_Float one, 
   Rox_Array2D_Float two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !median || !sum_square || !count_valid )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !res || !mask || !one || !two ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(one, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_check_size(two, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** dres = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** done = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &done, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dtwo = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dtwo, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dm, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double sum = 0.0;
   Rox_Sint count = 0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (dm[i][j] == 0) continue;

         sum += done[i][j] - dtwo[i][j];
         count++;
      }
   }

   if (count == 0) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   // mean = sum / (Rox_Float) count;

   Rox_Double sumsq = 0.0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         dres[i][j] = 0.0;

         if (dm[i][j] == 0) continue;

         dres[i][j] = done[i][j] - dtwo[i][j]; // dres[i][j] = done[i][j] - dtwo[i][j] - mean;
         sumsq += dres[i][j] * dres[i][j];
      }
   }

   *median = 0;
   *sum_square = sumsq;
   *count_valid = count;

function_terminate:
   return error;
}
