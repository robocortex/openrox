//==============================================================================
//
//    OPENROX   : File array2d_save.c
//
//    Contents  : Implementation of array2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_save.h"
#include "ansi_array2d_save.h"
#include "ansi_array2d_print.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_fprint_precision ( Rox_Array2D_Double input, FILE * file, Rox_Uint precision )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   char format[16];

   if ( (input==NULL) || (file==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint rows = 0, cols = 0;

   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sprintf(format, "%%.%d%c ", precision, 'f');

   fprintf(file, "Matrix (%ux%u):\n", rows, cols);
   for ( Rox_Sint i = 0 ; i < rows ; ++i)
   {
      for ( Rox_Sint j = 0 ; j < cols ; ++j)
      {
         fprintf(file, format, data[i][j]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_fprint ( FILE *file, const Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( (input==NULL) || (file==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         fprintf(file, "%32.32f ", data[i][j]);
      }
      fprintf(file, "\n");
   }
   
function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_double_fprint_pretty ( FILE * file, const Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( (input==NULL) || (file==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         fprintf(file, "%32.32f ", data[i][j]);
      }
      fprintf(file, "\n");
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_fprint ( FILE *file, const Rox_Array2D_Uint input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( (input==NULL) || (file==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uint_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         fprintf(file, "%u ", data[i][j]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_fprint ( FILE * file, const Rox_Array2D_Uchar input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( (input==NULL) || (file==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         fprintf(file, "%u ", data[i][j]);
      }
      fprintf(file, "\n");
   }

function_terminate:
   return error;
} 

Rox_ErrorCode rox_array2d_float_fprint ( FILE *file, const Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( (input==NULL) || (file==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** input_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &input_data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_fprint ( file, input_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_save (
   const Rox_Char * filename, 
   const Rox_Array2D_Double input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_fprint(file, input);

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_array2d_double_save_append ( 
   const Rox_Char * filename, 
   const Rox_Array2D_Double input 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "a");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_fprint(file, input);

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_array2d_float_save ( const Rox_Char * filename, const Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_fprint ( file, input );

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_array2d_uint_save(const char *filename, Rox_Array2D_Uint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_fprint(file, input);

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}

Rox_ErrorCode rox_array2d_uchar_save(const Rox_Char * filename, Rox_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_fprint(file, input);

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}

Rox_ErrorCode rox_array2d_double_save_pretty ( const Rox_Char * filename, Rox_Array2D_Double input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen ( filename, "w" );
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_fprint_pretty(file, input);

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}

Rox_ErrorCode rox_array2d_float_fscan ( Rox_Array2D_Float input, FILE * file )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Float ** data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef old
   // fread(data, sizeof(float), rows*cols, file);
   for ( Rox_Sint v = 0; v < rows; v++ )
   {
      for ( Rox_Sint u = 0; u < cols; u++ )
      {
         Rox_Sint nbr = 0;
         nbr = fscanf(file, "%f", &data[v][u]);
         if (nbr != 1) 
         { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }
      }
   }
#else

   error = rox_ansi_array2d_float_fscan ( data, rows, cols, file );
   ROX_ERROR_CHECK_TERMINATE ( error );

#endif

function_terminate: 
   return error;
}

Rox_ErrorCode rox_array2d_float_read ( const Rox_Array2D_Float input, const Rox_Char * filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "r");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_fscan ( input, file );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_array2d_double_read ( const Rox_Array2D_Double input, const Rox_Char * filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "r");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_double_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // fread(data, sizeof(float), rows*cols, file);
   for ( Rox_Sint v = 0; v < rows; v++ )
   {
      for ( Rox_Sint u = 0; u < cols; u++ )
      {
         Rox_Sint nbr = 0;         
         nbr = fscanf(file, "%lf", &data[v][u]);
         if (nbr != 1) 
         { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }
      }
   }

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}

Rox_ErrorCode rox_array2d_uint_read ( const Rox_Array2D_Uint input, const Rox_Char * filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;

   if ( (input==NULL) || (filename==NULL) )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "r");

   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uint_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_ansi_array2d_uint_fscan ( data, rows, cols, file );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // for ( Rox_Sint v = 0; v < rows; v++ )
   // {
   //    for ( Rox_Sint u = 0; u < cols; u++ )
   //    {
   //       Rox_Sint nbr = 0;         
   //       nbr = fscanf(file, "%u", &data[v][u]);
   //       if (nbr != 1) 
   //       { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }
   //    }
   // }

function_terminate: 
   if ( file != NULL ) fclose(file);

   return error;
}


