//==============================================================================
//
//    OPENROX   : File integral.c
//
//    Contents  : Implementation of image integral module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "integral.h"

Rox_ErrorCode rox_getValFromIntegral ( 
   Rox_Double * retour, 
   Rox_Double ** integral, 
   const int x, 
   const int y, 
   const int w, 
   const int h 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Get value for a given rect using an integral image as input
   // f(x,y,w,h) = f(x,y) + f(x+w,y+h) - f(x, y+h) - f(x + w, y)
   // Un-reachable f(.,.) are considered null

   if ( !integral )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   * retour = integral[y + h - 1][x + w - 1];

   if (y > 0)
   {
      if (x > 0)
      {
         *retour += integral[y - 1][x - 1];
      }

      *retour -= integral[y - 1][x + w - 1];
   }
   
   if (x > 0)
   {
      *retour -= integral[y + h - 1][x - 1];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_integral_sqr_double (
   Rox_Matrix I_count_int, 
   Rox_Matrix I_sum_int, 
   Rox_Matrix I_square_int, 
   Rox_Imask I_mask, 
   Rox_Array2D_Float I
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !I_count_int || !I_sum_int || !I_square_int || !I_mask || !I)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Float ** source   = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer( &source, I);
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Uint ** mask      = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &mask, I_mask);
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** I_count = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &I_count, I_count_int);
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** I_sum   = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &I_sum, I_sum_int);
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** I_square = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &I_square, I_square_int);
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Float * row_source = NULL;
   Rox_Uint * row_mask = NULL;
   Rox_Double * row_count = NULL;
   Rox_Double * row_sum = NULL;
   Rox_Double * row_square = NULL;
   Rox_Double * row_count_prev = NULL;
   Rox_Double * row_sum_prev = NULL;
   Rox_Double * row_square_prev = NULL;
      
   Rox_Sint rows = 0, cols = 0; 
   
   error = rox_array2d_float_get_size(&rows, &cols, I);
   ROX_ERROR_CHECK_TERMINATE( error ); 

   // Summing cells by cols
   for(Rox_Sint r = 0; r < rows; r++)
   {
      row_source  = source[r];
      row_mask = mask[r];
      row_count = I_count[r];
      row_sum = I_sum[r];
      row_square = I_square[r];
      
      if(row_mask[0] != 0)
      {
         row_count[0] = 1;
         row_sum[0] = row_source[0];
         row_square[0] = row_source[0] * row_source[0];
      }
      else
      {
         row_count[0] = 0;
         row_sum[0] = 0;
         row_square[0] = 0;
      }

      for(Rox_Sint c = 1; c < cols; c++)
      {
         if (row_mask)
         {
            row_count[c] = row_count[c-1] + 1;
            row_sum[c]  = row_source[c] + row_sum[c-1];
            row_square[c] = row_source[c] * row_source[c] + row_square[c-1];
         }
         else
         {
            row_count[c] = row_count[c-1];
            row_sum[c]  = row_source[c];
            row_square[c] = row_square[c-1];
         }
      }
   }

   // Summing rows
   for(Rox_Sint r = 1; r < rows; r++)
   {
      row_count = I_count[r];
      row_sum = I_sum[r];
      row_square = I_square[r];

      row_count_prev = I_count[r-1];
      row_sum_prev = I_sum[r-1];
      row_square_prev = I_square[r-1];

      for (Rox_Sint c = 0; c < cols; c++)
      {
         row_count[c] += row_count_prev[c];
         row_sum[c] += row_sum_prev[c];
         row_square[c] += row_square_prev[c];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_integral_sqr_double_nomask (
   Rox_Matrix I_count_int, 
   Rox_Matrix I_sum_int, 
   Rox_Matrix I_square_int, 
   Rox_Array2D_Float I
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !I_count_int || !I_sum_int || !I_square_int || !I)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Float ** source = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &source, I);
   ROX_ERROR_CHECK_TERMINATE( error ); 

   Rox_Double ** I_count = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &I_count, I_count_int);
   ROX_ERROR_CHECK_TERMINATE( error ); 

   Rox_Double ** I_sum = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &I_sum, I_sum_int);
   ROX_ERROR_CHECK_TERMINATE( error ); 

   Rox_Double ** I_square = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &I_square, I_square_int);
   ROX_ERROR_CHECK_TERMINATE( error ); 

   Rox_Float * row_source = NULL;
   Rox_Double * row_count = NULL;
   Rox_Double * row_sum = NULL;
   Rox_Double * row_square = NULL;
   Rox_Double * row_count_prev = NULL;
   Rox_Double * row_sum_prev = NULL;
   Rox_Double * row_square_prev = NULL;
      

   Rox_Sint rows = 0, cols = 0; 
   error = rox_array2d_float_get_size(&rows, &cols, I);
   ROX_ERROR_CHECK_TERMINATE( error );

   // Summing cells by cols
   for(Rox_Sint r = 0; r < rows; r++)
   {
      row_source  = source[r];
      row_count = I_count[r];
      row_sum = I_sum[r];
      row_square = I_square[r];
      
      row_count[0] = 1;
      row_sum[0] = row_source[0];
      row_square[0] = row_source[0] * row_source[0];

      for(Rox_Sint c = 1; c < cols; c++)
      {
         row_count[c] = row_count[c-1] + 1;
         row_sum[c]  = row_source[c] + row_sum[c-1];
         row_square[c] = row_source[c] * row_source[c] + row_square[c-1];
      }
   }

   // Summing rows
   for(Rox_Sint r = 1; r < rows; r++)
   {
      row_count = I_count[r];
      row_sum = I_sum[r];
      row_square = I_square[r];

      row_count_prev = I_count[r-1];
      row_sum_prev = I_sum[r-1];
      row_square_prev = I_square[r-1];

      for (Rox_Sint c = 0; c < cols; c++)
      {
         row_count[c] += row_count_prev[c];
         row_sum[c] += row_sum_prev[c];
         row_square[c] += row_square_prev[c];
      }
   }

function_terminate:
   return error;
}
