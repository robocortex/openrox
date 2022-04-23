//==============================================================================
//
//    OPENROX   : File array2d_print.c
//
//    Contents  : Implementation of array2d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_print.h"

#include <stdio.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_array2d_double_print ( Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** in = NULL;
   rox_array2d_double_get_data_pointer_to_pointer( &in, input);

   rox_log("Matrix (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         rox_log("%.16f, ", in[i][j]);

      }
      rox_log("\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_print ( Rox_Array2D_Uint input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uint_get_size(&rows, &cols, input);

   Rox_Uint ** in = NULL;
   rox_array2d_uint_get_data_pointer_to_pointer( &in, input);

   rox_log("Matrix (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         rox_log("%u, ", in[i][j]);
      }
      rox_log("\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_print ( Rox_Array2D_Uchar input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uchar_get_size(&rows, &cols, input);

   Rox_Uchar ** in = NULL;
   rox_array2d_uchar_get_data_pointer_to_pointer ( &in, input);

   rox_log("Matrix (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         rox_log("%u, ", in[i][j]);
      }
      rox_log("\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_print ( Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_float_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** input_data = NULL;
   rox_array2d_float_get_data_pointer_to_pointer( &input_data, input);

   rox_log("Matrix (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         rox_log("%.16f, ", input_data[i][j]);

      }
      rox_log("\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_print_precision(Rox_Array2D_Double input, Rox_Sint precision)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char format[16];
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **in = NULL;
   rox_array2d_double_get_data_pointer_to_pointer(&in, input);

   sprintf(format, "%%.%d%c ", precision, 'f');

   rox_log("Matrix (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         rox_log(format, in[i][j]);
      }
      rox_log("\n");
   }

function_terminate:
   return error;
}
