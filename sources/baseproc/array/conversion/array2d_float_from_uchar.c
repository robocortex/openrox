//==============================================================================
//
//    OPENROX   : File array2d_float_from_uchar.c
//
//    Contents  : Implementation of array2d_uchar_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_float_from_uchar.h"

#include <float.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_from_uchar ( 
   Rox_Array2D_Float output, const Rox_Array2D_Uchar input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D) output, (Rox_Array2D) input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** inp = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** out = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!inp || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (float) inp[i][j];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_from_uchar_normalize (
   Rox_Array2D_Float output, const Rox_Array2D_Uchar input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size ( (Rox_Array2D) output, (Rox_Array2D) input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** inp = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** out = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!inp || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (1.0f / 255.0f) * (float) inp[i][j];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_from_uchar_normalize_minmax ( Rox_Array2D_Float output, const Rox_Array2D_Uchar input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double min = DBL_MAX;
   Rox_Double max = -DBL_MAX;
   Rox_Sint cols = 0, rows = 0;
   Rox_Double scale = 0;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D) output, (Rox_Array2D) input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar **inp = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &inp, input  );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **out = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!inp || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (inp[i][j] < min) min = inp[i][j];
         if (inp[i][j] > max) max = inp[i][j];
      }
   }

   scale = 1.0 / (max - min);

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (float) (((float) inp[i][j] - min) * scale) ;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_new_from_uchar_normalize ( Rox_Array2D_Float * output, const Rox_Array2D_Uchar input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float ret = NULL;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_from_uchar_normalize ( ret, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   *output = ret;

function_terminate:
   if(error) rox_array2d_float_del(&ret);
   return error;
}

Rox_ErrorCode rox_array2d_float_new_from_uchar ( Rox_Array2D_Float * output, const Rox_Array2D_Uchar input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float ret = NULL;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size (&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_from_uchar ( ret, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   *output = ret;

function_terminate:
   if(error) rox_array2d_float_del(&ret);
   return error;
}

// This function should be in file array_float_from_uchar.c

int rox_ansi_array_float_from_uchar ( float * output, const unsigned char * input, const int size )
{
   int error = 0;

   for ( int k = 0; k < size; k++)
   {
      output[k] = (float) input[k];
   }

   return error;
}
