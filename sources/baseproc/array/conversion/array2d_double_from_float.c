//==============================================================================
//
//    OPENROX   : File array2d_double_from_float.c
//
//    Contents  : Implementation of array2d_double_from_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_double_from_float.h"

#include <float.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_from_float ( Rox_Array2D_Double output, const Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D) output, (Rox_Array2D) input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** in = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &in, input );

   Rox_Double ** out = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &out, output );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!in || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (double) in[i][j];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_from_float_normalize ( Rox_Array2D_Double output, const Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D) output, (Rox_Array2D) input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** in = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &in, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** out = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!in || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (1.0f / 255.0f) * (double) in[i][j];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_from_float_normalize_minmax(Rox_Array2D_Double output, Rox_Array2D_Float input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0;
   Rox_Double min = DBL_MAX;
   Rox_Double max = -DBL_MAX;
   Rox_Double scale = 0;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D) output, (Rox_Array2D) input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float  **in = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &in, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **out = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!in || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (in[i][j] < min) min = in[i][j];
         if (in[i][j] > max) max = in[i][j];
      }
   }

   scale = 1.0 / (max - min);

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (Rox_Double) (((Rox_Double) in[i][j] - min) * scale) ;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_from_float_buffer ( Rox_Array2D_Double output, Rox_Float * input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float * in = input;

   Rox_Double ** out = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!in || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (double) *in++;
      }
   }

function_terminate:
   return error;
}
