//==============================================================================
//
//    OPENROX   : File mean.c
//
//    Contents  : Implementation of mean module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mean.h"
#include <inout/system/errors_print.h>

// These two functions are useless: same duration time of ansi function
#ifdef VECTORISATION
int rox_avx_array2d_float_mean ( float ** res_data, float ** one_data, float ** two_data, int rows, int cols )
{
   int error = 0;   
   // union avx_vector buffer_8_bytes;

   __m256 avx_2 = _mm256_set1_ps(2);

   for ( int v = 0; v < rows; v++ )
   {
      float * ptr_one = one_data[v];
      float * ptr_two = two_data[v];
      float * ptr_res = res_data[v];

      for ( int u = 0; u < cols; u+=8 )
      {
         __m256 avx_one = _mm256_loadu_ps ( ptr_one );
         __m256 avx_two = _mm256_loadu_ps ( ptr_two );

         // Add two rows
         __m256 add = _mm256_add_ps ( avx_one, avx_two );

         // Divide by 2.0
         __m256 mean = _mm256_div_ps ( add, avx_2 );

         // Get the result
         _mm256_storeu_ps ( ptr_res, mean );

         // Increment pointers
         ptr_one += 8;
         ptr_two += 8;
         ptr_res += 8;
      }
   }
   return error;
}

int rox_sse_array2d_float_mean ( float ** res_data, float ** one_data, float ** two_data, int rows, int cols )
{
   int error = 0;   
   // union ssevector buffer_4_bytes;

   __m128 sse_2 = _mm_set_ps1(2);

   for ( int v = 0; v < rows; v++ )
   {
      float * ptr_one = one_data[v];
      float * ptr_two = two_data[v];
      float * ptr_res = res_data[v];

      for ( int u = 0; u < cols; u+=4 )
      {
         __m128 sse_one = _mm_loadu_ps ( ptr_one );
         __m128 sse_two = _mm_loadu_ps ( ptr_two );

         // Add two rows
         __m128 add = _mm_add_ps ( sse_one, sse_two );

         // Divide by 2.0
         __m128 mean = _mm_div_ps ( add, sse_2 );

         // Get the result
         _mm_storeu_ps ( ptr_res, mean );

         // Increment pointers
         ptr_one += 4;
         ptr_two += 4;
         ptr_res += 4;
      }
   }
   return error;
}
#endif

int rox_ansi_array2d_float_mean ( float ** res_data, float ** one_data, float ** two_data, int rows, int cols )
{
   int error = 0;

   for ( int i = 0; i < rows; i++)
   {
      for ( int j = 0; j < cols; j++)
      {
         res_data[i][j] = (one_data[i][j] + two_data[i][j]) * 0.5f;
      }
   }

   return error;
}

Rox_ErrorCode rox_array2d_float_mean ( Rox_Array2D_Float res, const Rox_Array2D_Float one, const Rox_Array2D_Float two )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !res )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   if ( !one || !two )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, res ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_float_get_stride ( &stride, res ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( one, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( two, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** res_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &res_data, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** one_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &one_data, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** two_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &two_data, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_mean ( res_data, one_data, two_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_sse_array2d_float_mean ( res_data, one_data, two_data, rows, cols );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_avx_array2d_float_mean ( res_data, one_data, two_data, rows, cols );
   // ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_mean ( Rox_Array2D_Double res, const Rox_Array2D_Double one, const Rox_Array2D_Double two )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(one, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(two, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **res_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &res_data, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **one_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &one_data, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** two_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &two_data, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         res_data[i][j] = (one_data[i][j] + two_data[i][j]) * 0.5;
      }
   }

function_terminate:
   return error;
}
