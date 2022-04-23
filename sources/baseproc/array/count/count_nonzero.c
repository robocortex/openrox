//==============================================================================
//
//    OPENROX   : File count_nonzero.c
//
//    Contents  : Implementation of count_nonzero module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "count_nonzero.h"
#include <inout/system/errors_print.h>

int rox_ansi_array_uint_matrix_count_nonzero_step_ogl  ( size_t * count, unsigned int * input_data, size_t input_rows, size_t input_cols, size_t step )
{
   int error = 0;
   
   size_t k = 0;

   for ( size_t r = 0; r < input_rows; r+=step )
   {
      size_t row_index = (input_rows-1-r)*input_cols; // Vertical flip 
      for ( size_t c = 0; c < input_cols; c+= step )
      {
         if ( input_data[row_index+c] > 0 ) k = k + 1;
      }
   }
   *count = k;

   return error;
}

int rox_ansi_array_uint_count_nonzero  ( size_t * count, unsigned int * input_data, size_t input_size )
{
   int error = 0;
   size_t k = 0;

   for ( size_t i = 0; i < input_size; i++)
   {
      if ( input_data[i] > 0 ) k = k + 1;
   }
   *count = k;

   return error;
}

int rox_ansi_array2d_uint_count_nonzero  ( size_t * count, unsigned int ** input_data, size_t input_rows, size_t input_cols  )
{
   int error = 0;
   size_t k = 0;

   for ( size_t r = 0; r < input_rows; r++)
   {
      for ( size_t c = 0; c < input_cols; c++)
      {
         if ( input_data[r][c] > 0 ) k = k + 1;
      }
   }
   *count = k;

   return error;
}

Rox_ErrorCode  rox_array2d_uint_count_nonzero ( Rox_Size * count, Rox_Array2D_Uint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input || !count ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size ( &rows, &cols, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** input_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &input_data, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Size k = 0;

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
          if (input_data[i][j] > 0) k = k + 1;
      }
   }

   *count = k;

function_terminate:
   return error;
}

int rox_ansi_array_float_count_nonzero  ( size_t * count, float * input_data, size_t input_size )
{
   int error = 0;
   size_t k = 0;

   for ( size_t i = 0; i < input_size; i++)
   {
      if ( input_data[i] > 0 ) k = k + 1;
   }
   *count = k;

   return error;
}
