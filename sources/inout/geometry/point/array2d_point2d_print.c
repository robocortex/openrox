//==============================================================================
//
//    OPENROX   : File array2d_point2d_print.c
//
//    Contents  : Implementation of point2d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "array2d_point2d_print.h"
#include <system/errors/errors.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_point2d_double_print(Rox_Array2D_Point2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_double_get_size(&rows, &cols, input);

   rox_log("Array (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Point2D_Double_Struct result;

         error = rox_array2d_point2d_double_get_value(&result, input, r, c);

         rox_log("%.16f %.16f ; ", result.u, result.v);

      }
      rox_log("\n");
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_point2d_float_print ( Rox_Array2D_Point2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_float_get_size(&rows, &cols, input);

   rox_log("Array (%lux%lu):\r\n", (long unsigned int) rows, (long unsigned int) cols);
   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Point2D_Float_Struct result;

         error = rox_array2d_point2d_float_get_value(&result, input, r, c);

         rox_log("%.16f %.16f ; ", result.u, result.v);

      }
      rox_log("\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_sshort_fprint ( FILE *file, const Rox_Array2D_Point2D_Sshort input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!file || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_point2d_sshort_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort_Struct **data = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         fprintf(file, "%d ", data[i][j].u);
         fprintf(file, "%d ", data[i][j].v);
      }
   }
   fprintf(file, "\n");
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_sshort_save ( const Rox_Char * filename, const Rox_Array2D_Point2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( !input || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_point2d_sshort_fprint(file, input);

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}
