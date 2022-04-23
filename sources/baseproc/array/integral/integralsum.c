//==============================================================================
//
//    OPENROX   : File integralsum.c
//
//    Contents  : Implementation of integralsum module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "integralsum.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_integralsum ( Rox_Array2D_Double intsum, Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double * row_sum = NULL;
   Rox_Double * row_sum_prev = NULL;

   if (!intsum || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(intsum, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** di = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &di, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** ds = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &ds, intsum );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Summing cells by cols
   for ( Rox_Sint r=0; r < rows; r++)
   {
      Rox_Double * row_source  = di[r];
      row_sum = ds[r];

      row_sum[0] = row_source[0];

      for ( Rox_Sint c=1; c<cols; c++)
      {
         row_sum[c]  = (Rox_Float)row_source[c] + (Rox_Float)row_sum[c-1];
      }
   }

   // Summing rows
   for ( Rox_Sint r = 1; r < rows; r++)
   {
      row_sum = ds[r];
      row_sum_prev = ds[r-1];

      for ( Rox_Sint c=0; c < cols; c++)
      {
         row_sum[c] += row_sum_prev[c];
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_float_integralsum(Rox_Array2D_Float intsum, Rox_Array2D_Float input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Float * row_sum = NULL;
   Rox_Float * row_sum_prev = NULL;

   if (!intsum || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( intsum, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** di = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &di, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &ds, intsum );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Summing cells by cols
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      Rox_Float * row_source  = di[r];
      row_sum = ds[r];

      row_sum[0] = row_source[0];

      for ( Rox_Sint c = 1; c < cols; c++)
      {
         row_sum[c]  = (Rox_Float)row_source[c] + (Rox_Float)row_sum[c-1];
      }
   }

   // Summing rows
   for ( Rox_Sint r=1; r < rows; r++)
   {
      row_sum = ds[r];
      row_sum_prev = ds[r-1];

      for ( Rox_Sint c=0; c < cols; c++)
      {
         row_sum[c] += row_sum_prev[c];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_sint_integralsum ( Rox_Array2D_Slint intsum, Rox_Array2D_Sint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Slint * row_sum = NULL;
   Rox_Slint * row_sum_prev = NULL;

   if (!intsum || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_sint_get_size ( &rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_slint_check_size ( intsum, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint ** di = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &di, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Slint ** ds = NULL;
   error = rox_array2d_slint_get_data_pointer_to_pointer ( &ds, intsum );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Summing cells by cols
   for ( Rox_Sint r=0; r < rows; r++)
   {
      Rox_Sint * row_source  = di[r];
      row_sum = ds[r];

      row_sum[0] = row_source[0];

      for ( Rox_Sint c = 1; c < cols; c++)
      {
         row_sum[c]  = ((Rox_Slint)row_source[c]) + row_sum[c-1];
      }
   }

   // Summing rows
   for ( Rox_Sint r = 1; r < rows; r++)
   {
      row_sum = ds[r];
      row_sum_prev = ds[r-1];

      for ( Rox_Sint c = 0; c < cols; c++)
      {
         row_sum[c] += row_sum_prev[c];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_integralsum(Rox_Array2D_Uint intsum, Rox_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint * row_sum = NULL;
   Rox_Uint * row_sum_prev = NULL;

   if (!intsum || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( intsum, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar **di = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &di, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint **ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &ds, intsum );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Summing cells by cols
   for ( Rox_Sint r=0; r < rows; r++)
   {
      Rox_Uchar * row_source  = di[r];
      row_sum = ds[r];

      row_sum[0] = row_source[0];

      for ( Rox_Sint c=1; c<cols; c++)
      {
         row_sum[c]  = ((Rox_Uint)row_source[c]) + row_sum[c-1];
      }
   }

   // Summing rows
   for ( Rox_Sint r=1; r < rows; r++)
   {
      row_sum = ds[r];
      row_sum_prev = ds[r-1];

      for ( Rox_Sint c=0; c < cols; c++)
      {
         row_sum[c] += row_sum_prev[c];
      }
   }

function_terminate:
   return error;
}
