//==============================================================================
//
//    OPENROX   : File l2_error.c
//
//    Contents  : Implementation of l2_error module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "l2_error.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_l2_error (
   Rox_Double * error_out, 
   Rox_Array2D_Float error_img, 
   Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!error_out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   if (!error_img) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   if (!mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, error_img); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** derror = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &derror, error_img );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double sum = 0.0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (dm[i][j] == 0) continue;

         sum += derror[i][j] * derror[i][j];
      }
   }

   *error_out = sqrt(sum);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_difference_l2_norm (
   Rox_Double * l2_error,
   const Rox_Array2D_Double array2d_1,
   const Rox_Array2D_Double array2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!l2_error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_1_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Double sum = 0.0;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Double dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         sum += dif*dif;
      }
   }

   *l2_error = sqrt(sum);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_difference_l2_norm (
   Rox_Float * l2_error,
   const Rox_Array2D_Float array2d_1,
   const Rox_Array2D_Float array2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!l2_error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_1_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_2_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Float sum = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         sum += dif*dif;
      }
   }

   *l2_error = sqrtf(sum);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_difference_l2_norm (
   Rox_Float * l2_error,
   const Rox_Array2D_Uchar array2d_1,
   const Rox_Array2D_Uchar array2d_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!l2_error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_1_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_2_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float sum = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = (Rox_Float) array2d_2_data[r][c] - (Rox_Float) array2d_1_data[r][c];

         sum += dif*dif;
      }
   }

   *l2_error = sqrtf(sum);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_difference_l2_norm_mask (
   Rox_Double * l2_error,
   const Rox_Array2D_Double array2d_1,
   const Rox_Array2D_Double array2d_2,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!l2_error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_1_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** array2d_2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double sum = 0.0;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Double dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            sum += dif*dif;
         }
      }
   }

   *l2_error = sqrt(sum);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_difference_l2_norm_mask (
   Rox_Float * l2_error,
   const Rox_Array2D_Float array2d_1,
   const Rox_Array2D_Float array2d_2,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!l2_error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_1_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** array2d_2_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float sum = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            sum += dif*dif;
         }
      }
   }

   *l2_error = sqrtf(sum);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_difference_l2_norm_mask (
   Rox_Float * l2_error,
   const Rox_Array2D_Uchar array2d_1,
   const Rox_Array2D_Uchar array2d_2,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!l2_error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!array2d_2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_1_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_1_data, array2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** array2d_2_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &array2d_2_data, array2d_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float sum = 0.0f;
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Float dif = (Rox_Float) array2d_2_data[r][c] - (Rox_Float) array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            sum += dif*dif;
         }
      }
   }

   *l2_error = sqrtf(sum);

function_terminate:
   return error;
}
