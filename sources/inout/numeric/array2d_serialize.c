//==============================================================================
//
//    OPENROX   : File array2d_serialize.c
//
//    Contents  : Implementation of array2d_serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_serialize.h"

#include <baseproc/geometry/point/point2d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_serialize_binary (const char * filename, Rox_Array2D_Uint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;
   size_t write_res = 0;

   if ( !filename ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "wb");
   if ( !out ) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** data = NULL;
   rox_array2d_uint_get_data_pointer_to_pointer(&data, input);

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uint_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   write_res = fwrite(&cols, sizeof(Rox_Uint), 1, out);
   if (write_res != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   write_res = fwrite(&rows, sizeof(Rox_Uint), 1, out);
   if (write_res != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      write_res = fwrite(data[i], sizeof(Rox_Uint), cols, out);
      if (write_res != cols) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  }


function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_array2d_uint_deserialize_binary(Rox_Array2D_Uint output, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;
   Rox_Uint ocols, orows;
   size_t read_res = 0;

   if (!filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   in = fopen(filename, "rb");
   if (!in) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** data = NULL;
   rox_array2d_uint_get_data_pointer_to_pointer( &data, output);

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( ocols != cols || orows != rows )
   {
      fclose(in);
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Uint), cols, in);
      if ( read_res != cols ) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

function_terminate:
   if(in) fclose(in);   
   return error;
}

Rox_ErrorCode rox_array2d_uchar_serialize(FILE * out, Rox_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** data = NULL;
   rox_array2d_uchar_get_data_pointer_to_pointer( &data, input);

   Rox_Sint cols = 0, rows = 0; 

   error = rox_array2d_uchar_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   fwrite(&cols, sizeof(Rox_Uint), 1, out);
   fwrite(&rows, sizeof(Rox_Uint), 1, out);

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      fwrite(data[i], sizeof(Rox_Uchar), cols, out);
   }   
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_serialize_binary(const char * filename, Rox_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   out = fopen(filename, "wb");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   CHECK_ERROR_TERMINATE(rox_array2d_uchar_serialize(out, input))

function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_array2d_uchar_deserialize_binary(Rox_Array2D_Uchar output, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

   if (!filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   in = fopen(filename, "rb");
   if (!in) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   CHECK_ERROR_TERMINATE(rox_array2d_uchar_deserialize(output, in))

function_terminate:
   if(in) fclose(in);
   
   return error;
}

Rox_ErrorCode rox_array2d_uchar_deserialize(Rox_Array2D_Uchar output, FILE * in)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols, rows, i;
   Rox_Uint ocols, orows;
   Rox_Uchar ** data;
   size_t read_res = 0;
   
   //file handling
   if (!in) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   //read our data
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &data, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error =  rox_array2d_uchar_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if (read_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if (read_res != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   if (ocols != cols || orows != rows)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
 
   for (i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Uchar), cols, in);
      if(read_res != cols) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_serialize(FILE* out, Rox_Array2D_Float input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }   
   
   Rox_Float ** data = NULL;
   rox_array2d_float_get_data_pointer_to_pointer( &data, input);
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   fwrite(&cols, sizeof(Rox_Uint), 1, out);
   fwrite(&rows, sizeof(Rox_Uint), 1, out);
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      fwrite(data[i], sizeof(Rox_Float), cols, out);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_deserialize(Rox_Array2D_Float output, FILE* in)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ocols, orows;
   size_t read_res = 0;
   
   if (!in) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   Rox_Float ** data = NULL;
   rox_array2d_float_get_data_pointer_to_pointer( &data, output);

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   if (ocols != cols || orows != rows)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error); }
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Float), cols, in);
      if ( read_res != cols) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error); }
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_serialize_binary(const char * filename, Rox_Array2D_Float input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;
   
   if (!filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   out = fopen(filename, "wb");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   
   CHECK_ERROR_TERMINATE(rox_array2d_float_serialize(out, input))
   
function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_array2d_float_deserialize_binary(Rox_Array2D_Float output, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

   if (!filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   in = fopen(filename, "rb");
   if (!in) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   
   CHECK_ERROR_TERMINATE(rox_array2d_float_deserialize(output, in))

function_terminate:
   if(in) fclose(in);
   return error;
}

Rox_ErrorCode rox_array2d_double_serialize(FILE* out, Rox_Array2D_Double input)
{  
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   //tests
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
      
   //read data
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   fwrite(&cols, sizeof(Rox_Uint), 1, out);
   fwrite(&rows, sizeof(Rox_Uint), 1, out);
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      fwrite(data[i], sizeof(Rox_Double), cols, out);
   }   
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_deserialize(Rox_Array2D_Double output, FILE* in)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ocols, orows;
   size_t read_res = 0;
   
   // Tests
   if (!in) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Read data
   Rox_Double ** data = NULL;
   rox_array2d_double_get_data_pointer_to_pointer( &data, output);
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
  
   if (ocols != cols || orows != rows)
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Double), cols, in);
      if(read_res != cols) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }
    
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_float_serialize(FILE* out, Rox_Array2D_Point2D_Float input)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   // Tests IO
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   // Write data
   Rox_Point2D_Float * data = NULL;
   error = rox_array2d_point2d_float_get_data_pointer_to_pointer ( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_float_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   fwrite(&cols, sizeof(Rox_Uint), 1, out);
   fwrite(&rows, sizeof(Rox_Uint), 1, out);
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      fwrite(data[i], sizeof(Rox_Point2D_Float_Struct), cols, out);
   }  
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_float_deserialize(Rox_Array2D_Point2D_Float output, FILE* in)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ocols, orows;
   size_t read_res = 0;
   
   //Tests IO
   if (!in) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   //Read data
   Rox_Point2D_Float * data = NULL;
   error = rox_array2d_point2d_float_get_data_pointer_to_pointer ( &data, output);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_float_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if (read_res != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if (read_res != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   if (ocols != cols || orows != rows)
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Point2D_Float_Struct), cols, in);
      if(read_res != cols) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
  }
     
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_sshort_serialize(FILE* out, Rox_Array2D_Point2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
      size_t write_res = 0;

   //Tests IO
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
      
   //Write data
   Rox_Point2D_Sshort * data = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_point2d_sshort_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   write_res = fwrite(&cols, sizeof(Rox_Uint), 1, out);

   if (write_res != 1) 
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   write_res = fwrite(&rows, sizeof(Rox_Uint), 1, out);
   if (write_res != 1) 
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
  
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      write_res = fwrite(data[i], sizeof(Rox_Point2D_Sshort_Struct), cols, out);
    if(write_res != cols) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
  }  
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_point2d_sshort_deserialize(Rox_Array2D_Point2D_Sshort output, FILE* in)
{    
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ocols, orows;
   size_t read_res = 0;
   
   //Tests IO
   if (!in) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   if (!output) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   //Read data
   Rox_Point2D_Sshort * data = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &data, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_sshort_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if ( read_res != 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if ( read_res != 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (ocols != cols || orows != rows)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Point2D_Sshort_Struct), cols, in);
      if (read_res != cols) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  }
     
function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_sshort_serialize(FILE* out, Rox_Array2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   size_t write_res = 0;
 
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sshort ** data = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_sshort_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   write_res = fwrite(&cols, sizeof(Rox_Uint), 1, out);
   if ( write_res != 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   write_res = fwrite(&rows, sizeof(Rox_Uint), 1, out);
   if ( write_res != 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      write_res = fwrite(data[i], sizeof(Rox_Sshort), cols, out);
      if ( write_res != cols ) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
  }
      
function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_sshort_serialize_binary(const char * filename, Rox_Array2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   out = fopen(filename, "wb");
   if (!out) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   CHECK_ERROR_TERMINATE(rox_array2d_sshort_serialize(out, input))

function_terminate:
   if (out) fclose(out);   
   return error;
}

Rox_ErrorCode rox_array2d_sshort_deserialize(Rox_Array2D_Sshort output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ocols, orows;
   size_t read_res = 0;

   Rox_Sshort ** data = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer (&data, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_sshort_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = fread(&ocols, sizeof(Rox_Uint), 1, in);
   if ( read_res != 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   read_res = fread(&orows, sizeof(Rox_Uint), 1, in);
   if ( read_res != 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (ocols != cols || orows != rows)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      read_res = fread(data[i], sizeof(Rox_Sshort), cols, in);
      if (read_res != cols) 
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_sshort_deserialize_binary(Rox_Array2D_Sshort output, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

   if (!filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   in = fopen(filename, "rb");
   if (!in) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }   
   
   CHECK_ERROR_TERMINATE(rox_array2d_sshort_deserialize(output, in))

function_terminate:   
   if(in) fclose(in);   

   return ROX_ERROR_NONE;
}

