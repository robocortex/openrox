//==============================================================================
//
//    OPENROX   : File array_save.c
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

#include "array_save.h"

#include <stdio.h>

#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array_uint_file_write ( FILE * file, Rox_Uint * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!file || !input_data)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (input_size < 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < input_size; i++ )
   {
      fprintf(file, "%d ", input_data[i]);
   }
   fprintf(file, "\n");

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_float_file_write ( FILE * file, Rox_Float * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (input_size < 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < input_size; i++)
   {
      fprintf(file, "%.8f ", input_data[i]);
   }
   fprintf(file, "\n");

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_double_file_write ( FILE *file, Rox_Double * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (input_size <= 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < input_size; i++)
   {
      fprintf(file, "%.16f ", input_data[i]);
   }
   fprintf(file, "\n");

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_uint_save ( const Rox_Char * filename, Rox_Uint * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "w");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_uint_file_write(file, input_data, input_size);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (file != NULL) fclose(file);
   return error;
}

Rox_ErrorCode rox_array_float_save ( const Rox_Char * filename, Rox_Float * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "w");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_float_file_write(file, input_data, input_size);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (file != NULL) fclose(file);
   return error;
}

Rox_ErrorCode rox_array_double_save ( const Rox_Char * filename, Rox_Double * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "w");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_double_file_write ( file, input_data, input_size );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (file != NULL) fclose(file);
   return error;
}

Rox_ErrorCode rox_array_double_save_append ( const Rox_Char * filename, Rox_Double * input_data, Rox_Sint input_size )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "a");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_double_file_write ( file, input_data, input_size );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (file != NULL) fclose(file);
   return error;
}

// ---------------------------------------------------------------------------------------------------

Rox_ErrorCode rox_array_uchar_file_write_as_array2d ( FILE * file, Rox_Uchar * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Size r = 0; r < input_rows; r++ )
   {
      Rox_Size row = input_cols * r;
      for ( Rox_Size c = 0; c < input_cols; c++ )
      {
         fprintf(file, "%u ", input_data[row + c]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_uint_file_write_as_array2d ( FILE * file, Rox_Uint * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Size r = 0; r < input_rows; r++ )
   {
      Rox_Size row = input_cols * r;
      for ( Rox_Size c = 0; c < input_cols; c++ )
      {      
         fprintf(file, "%u ", input_data[row + c]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_float_file_write_as_array2d ( FILE * file, Rox_Float * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Size r = 0; r < input_rows; r++ )
   {
      Rox_Size row = input_cols * r;
      for ( Rox_Size c = 0; c < input_cols; c++ )
      {
         fprintf(file, "%.8f ", input_data[row + c]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_double_file_write_as_array2d ( FILE * file, Rox_Double * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Size r = 0; r < input_rows; r++ )
   {
      Rox_Size row = input_cols * r;
      for ( Rox_Size c = 0; c < input_cols; c++ )
      {
         fprintf(file, "%.16f ", input_data[row + c]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_float_padded_file_write_as_array2d ( FILE * file, Rox_Float * input_data, Rox_Size input_rows, Rox_Size input_cols, Rox_Size input_used )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (file == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (input_data == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Size r = 0; r < input_rows; r++ )
   {
      Rox_Size row = input_cols * r;
      for ( Rox_Size c = 0; c < input_used; c++ )
      {
         fprintf(file, "%.8f ", input_data[row + c]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_uchar_save_as_array2d ( const Rox_Char * filename, Rox_Uchar * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "w");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_uchar_file_write_as_array2d ( file, input_data, input_rows, input_cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_array_uint_save_as_array2d ( const Rox_Char * filename, Rox_Uint * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "w");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_uint_file_write_as_array2d ( file, input_data, input_rows, input_cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (file != NULL) fclose(file);

   return error;
}

Rox_ErrorCode rox_array_float_save_as_array2d ( const Rox_Char * filename, Rox_Float * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE * file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_float_file_write_as_array2d ( file, input_data, input_rows, input_cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if ( file != NULL ) fclose(file);

   return error;
}

Rox_ErrorCode rox_array_double_save_as_array2d ( const Rox_Char * filename, Rox_Double * input_data, Rox_Size input_rows, Rox_Size input_cols )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE *file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_double_file_write_as_array2d ( file, input_data, input_rows, input_cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_array_float_padded_save_as_array2d ( const Rox_Char * filename, Rox_Float * input_data, Rox_Size input_rows, Rox_Size input_cols, Rox_Size input_used )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE * file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array_float_padded_file_write_as_array2d ( file, input_data, input_rows, input_cols, input_used );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if ( file != NULL ) fclose(file);

   return error;
}