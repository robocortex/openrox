//============================================================================
//
//    OPENROX   : File pgmfile.c
//
//    Contents  : Implementation of pgmfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "pgmfile.h"

#include <stdio.h>
#ifdef ROX_USE_CTYPE
#include <ctype.h>
#endif
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pgm_strip_comments(FILE * input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint c = 0;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   do
   {
      //Get a character
      c = getc(input);
      if (c == '#')
      {
         //If comment, skip until next line / EOF
         do
         {
            c = getc(input);
         }
         while ((c != EOF) && (c != '\n'));
      }
   }
   while ((c == EOF) || isspace(c));

   // Replace last character in stream 
   ungetc(c, input);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pgm_read_header(FILE * input, Rox_Sint * rowsp, Rox_Sint * colsp, Rox_Sint * maxp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint c = 0;
   Rox_Sint cols = 0;
   Rox_Sint rows = 0;
   Rox_Sint max = 0;

   if (!input) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (!rowsp) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (!colsp) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (!maxp)  { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test magic number 
   if (fgetc(input) != 'P' || fgetc(input) != '5')
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Read PGM header 
   error = rox_pgm_strip_comments(input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get width 
   if (fscanf(input, "%d", &cols) < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_pgm_strip_comments(input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get height 
   if (fscanf(input, "%d", &rows) < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_pgm_strip_comments(input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get maximal value 
   if (fscanf(input, "%d", &max) < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get last space (typically LineFeed) 
   c = getc(input);
   if (!isspace(c))
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rows < 1 || cols < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (max > 255 || max < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *rowsp = rows;
   *colsp = cols;
   *maxp  = max;

function_terminate:
   return error;
}

Rox_ErrorCode rox_pgm_save_header(FILE * output, Rox_Sint rows, Rox_Sint cols)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fprintf(output, "P5\n%u %u\n%u\n", cols, rows, 255u);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pgm_read_content(Rox_Uchar ** out, FILE * inp, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!inp) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Sint iter = 0; iter < rows; iter++)
   {
      Rox_Size readcount = fread(out[iter], sizeof(Rox_Uchar), cols, inp);
      if (readcount != cols)
      { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_pgm_read_content_float_normalize(Rox_Float ** out, FILE * inp, const Rox_Sint cols, const Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!inp) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar * out_uchar = (Rox_Uchar *) rox_memory_allocate(sizeof(Rox_Uchar), cols);

   for (Rox_Sint iter = 0; iter < rows; iter++)
   {
      Rox_Size readcount = fread(&out_uchar, sizeof(Rox_Uchar), cols, inp);
      
      if (readcount != cols)
      { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
      for (Rox_Sint c = 0; c < cols; c++)
		{
      	out[iter][c] = (1.0f / 255.0f) * ((Rox_Float) out_uchar[c]);
		}
   }

   rox_memory_delete(out_uchar);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pgm_write_content(FILE * out, Rox_Uchar ** inp, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!out || !inp)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Size write_count = fwrite(inp[i], sizeof(Rox_Uchar), cols, out);
      if (write_count != cols * sizeof(Rox_Uchar))
      {
         error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_pgm_write_content_contiguous(FILE * out, Rox_Uchar * inp, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!out || !inp)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Size write_count = fwrite(inp, sizeof(Rox_Uchar), cols*rows, out);
   if (write_count != cols * rows * sizeof(Rox_Uchar))
   {
      error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_new_pgm ( Rox_Array2D_Uchar * out, const char * path )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar ret = NULL;
   FILE * pgmfile = NULL;

   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *out = NULL;

   pgmfile = fopen(path, "rb");
   if (!pgmfile) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0, maxval = 0;
   error = rox_pgm_read_header ( pgmfile, &rows, &cols, &maxval );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &data, ret );
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   error = rox_pgm_read_content ( data, pgmfile, cols, rows );
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   *out = ret;

function_terminate:
   if(error) rox_array2d_uchar_del(&ret);
   if(pgmfile) fclose(pgmfile);
   return error;
}

Rox_ErrorCode rox_array2d_uchar_read_pgm(Rox_Array2D_Uchar inp, const char *path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * pgmfile = NULL;

   if (!path || !inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pgmfile = fopen(path, "rb");
   if (!pgmfile) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows, cols, maxval;
   error = rox_pgm_read_header(pgmfile, &rows, &cols, &maxval);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width  = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != height || cols != width)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&data, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pgm_read_content(data, pgmfile, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:  
   if(pgmfile) fclose(pgmfile);
   return error;
}

Rox_ErrorCode rox_array2d_uint_rgba_new_pgm(Rox_Array2D_Uint * out, const char *path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uint ret = NULL;
   Rox_Array2D_Uchar source = NULL;

   if(!out || !path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *out = NULL;

   error = rox_array2d_uchar_new_pgm(&source, path); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint rows = 0, cols = 0;   
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxgray_to_roxrgba(ret, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *out = ret;

function_terminate:
   rox_array2d_uchar_del(&source);
   // Delete only if an error occurs
   if(error) rox_array2d_uint_del(&ret);
   return error;
}

Rox_ErrorCode rox_array2d_uint_rgba_read_pgm(Rox_Array2D_Uint out, const char * path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar source = NULL;

   if(!out || !path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_new_pgm(&source, path); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxgray_to_roxrgba(out, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   rox_array2d_uchar_del(&source);
   return error;
}

Rox_ErrorCode rox_array2d_uchar_save_pgm(const char * path, Rox_Array2D_Uchar inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE* file = NULL;

   if(!path || !inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** din  = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &din, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   file = fopen(path, "wb");
   if (!file) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_pgm_save_header(file, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pgm_write_content(file, din, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(file) fclose(file);
   return error;
}

Rox_ErrorCode rox_array2d_uint_rgba_save_pgm(const char * path, Rox_Array2D_Uint inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar source = NULL;

   if (!path || !inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&source, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_roxrgba_to_roxgray(source, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_save_pgm(path, source);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:   
   rox_array2d_uchar_del(&source);
   return error;
}

Rox_ErrorCode rox_array2d_float_normalize_read_pgm(Rox_Array2D_Float inp, const char *path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * pgmfile = NULL;

   if (!path || !inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pgmfile = fopen(path, "rb");
   if (!pgmfile) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows, cols, maxval;
   error = rox_pgm_read_header(pgmfile, &rows, &cols, &maxval);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   Rox_Sint width = 0, height = 0;
   error = rox_array2d_float_get_size(&height, &width, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != height || cols != width)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float ** data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&data, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pgm_read_content_float_normalize(data, pgmfile, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (pgmfile) fclose(pgmfile);
   return error;
}
