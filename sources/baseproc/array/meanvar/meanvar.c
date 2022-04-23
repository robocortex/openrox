//==============================================================================
//
//    OPENROX   : File meanvar.c
//
//    Contents  : Implementation of meanvar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "meanvar.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_meanvar ( Rox_Double * mean, Rox_Double * variance, Rox_Array2D_Float input, Rox_Array2D_Uint mask )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double sum = 0.0, lmean=0.0, delta=0.0;
   Rox_Size count = 0;

   if (!input || !mask || !mean || !variance) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_float_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** input_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &input_data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   sum = 0.0;
   count = 0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (!mask_data[i][j]) continue;

         sum += input_data[i][j];
         count++;
      }
   }

   if (count == 0) *mean = 0;
   else
   {
      lmean = sum / (Rox_Double)count;
      *mean = lmean;
   }

   sum = 0.0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (!mask_data[i][j]) continue;

         delta = input_data[i][j] - lmean;
         sum += (delta * delta);
      }
   }

   lmean = sum / (Rox_Double)(count-1);
   *variance = lmean;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_meanvar ( Rox_Double * mean, Rox_Double * variance, Rox_Array2D_Double input, Rox_Array2D_Uint mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double sum =0.0, lmean=0.0, delta=0.0;
   Rox_Size count=0;

   if (!input || !mask || !mean || !variance) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** input_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &input_data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   sum = 0.0;
   count = 0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (!mask_data[i][j]) continue;

         sum += input_data[i][j];
         count++;
      }
   }

   if (count == 0) *mean = 0;
   else
   {
      lmean = sum / (Rox_Double)count;
      *mean = lmean;
   }

   sum = 0.0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (!mask_data[i][j]) continue;

         delta = input_data[i][j] - lmean;
         sum += (delta * delta);
      }
   }

   lmean = sum / (Rox_Double)(count-1);
   *variance = lmean;

function_terminate:
   return error;
}
