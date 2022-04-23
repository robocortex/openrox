//==============================================================================
//
//    OPENROX   : File ppmfile.c
//
//    Contents  : Implementation of ppmfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ppmfile.h"

#include <stdio.h>
#ifdef ROX_USE_CTYPE
  #include <ctype.h>
#endif
#include <baseproc/image/convert/rgb_to_roxrgba.h>
#include <baseproc/image/convert/roxrgba_to_rgb.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ppm_strip_comments(FILE * input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint c = 0;

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   do
   {
      //Get a character
      c = getc(input);
      if(c == '#')
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

Rox_ErrorCode rox_ppm_save_header(FILE* fp, Rox_Sint rows, Rox_Sint cols)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!fp)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fprintf(fp, "P6\n%u %u\n%u\n", cols, rows, 255u);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ppm_read_header(FILE* input, Rox_Sint* rowsp, Rox_Sint* colsp, Rox_Sint* maxp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint c;
   Rox_Sint cols = 0;
   Rox_Sint rows = 0;
   Rox_Sint max = 0;

   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!rowsp) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!colsp) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!maxp)  {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Test magic number
   if (fgetc(input) != 'P' || fgetc(input) != '6')
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read PGM header
   error = rox_ppm_strip_comments(input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get width
   if(fscanf(input, "%d", &cols) < 1)
   {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ppm_strip_comments(input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get height
   if (fscanf(input, "%d", &rows) < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ppm_strip_comments(input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get maximal value
   if(fscanf(input, "%d", &max) < 1)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get last space (typically LineFeed)
   c = getc(input);
   if(!isspace(c))
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

Rox_ErrorCode rox_ppm_read_content(Rox_Uchar * out, FILE * in, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!in)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Size readcount = fread(out, sizeof(Rox_Uchar), 3 * rows * cols, in);
   if (readcount != 3 * rows * cols)
   {
     error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ppm_write_content(FILE * out, Rox_Uchar *in, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !out ) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !in ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fwrite(in, sizeof(Rox_Uchar), 3*rows*cols, out);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ppm_write_content_rgba(FILE * out, Rox_Uint *in, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !out )
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !in ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (size_t i = 0; i < 4 * rows*cols; i+=4)
   {
      fwrite(in, sizeof(Rox_Uchar), 3, out);
      in++;
   }

function_terminate:
   return error;

}

Rox_ErrorCode rox_image_rgba_new_read_ppm ( Rox_Image_RGBA * out, const char * path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * ppmfile = NULL;
   Rox_Sint rows = 0, cols = 0, maxval = 0;
   Rox_Image_RGBA output = NULL;
   Rox_Uchar *rgb = NULL;

   if ( !out ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !path ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ppmfile = fopen ( path, "rb" );
   if (!ppmfile) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_ppm_read_header ( ppmfile, &rows, &cols, &maxval );
   ROX_ERROR_CHECK_TERMINATE ( error );

   rgb = (Rox_Uchar*) rox_memory_allocate ( rows*cols*3, 1 );
   if ( rgb == 0 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_rgba_new ( &output, cols, rows );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ppm_read_content ( rgb, ppmfile, cols, rows );
   if (error)
   {
      rox_image_rgba_del ( &output );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_rgb_to_roxrgba ( output, rgb, cols * 3 );
   if (error)
   {
      rox_image_rgba_del ( &output );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *out = output;

function_terminate:
   if (ppmfile)
      fclose(ppmfile);
   if (rgb)
      rox_memory_delete(rgb);
   return error;
}

Rox_ErrorCode rox_image_rgba_read_ppm(Rox_Image_RGBA in, const char * path)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * ppmfile = NULL;
   Rox_Uchar * rgb = NULL;

   if (!path || !in){ error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ppmfile = fopen(path, "rb");
   if (!ppmfile){ error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0, maxval = 0;
   error = rox_ppm_read_header(ppmfile, &rows, &cols, &maxval);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(rows != height || cols != width)
   {
      error = ROX_ERROR_INVALID_VALUE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   rgb = (Rox_Uchar*) rox_memory_allocate(rows*cols*3, 1);
   if(rgb == 0)
   {
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   }

   error = rox_ppm_read_content(rgb, ppmfile, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Conversion from RGB to ROX_RGBA
   error = rox_rgb_to_roxrgba(in, rgb, cols * 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (ppmfile)
      fclose(ppmfile);
   if (rgb)
      rox_memory_delete(rgb);
   return error;
}


Rox_ErrorCode rox_image_rgba_save_noalloc_ppm(const Rox_Char * path, Rox_Image_RGBA in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * file = NULL;

   if ( !path || !in ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_rgba_get_size(&rows, &cols, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   file = fopen(path, "wb");
   if (!file) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ppm_save_header(file, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer(&data, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ppm_write_content_rgba(file, data[0], cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (file)
      fclose(file);
   return error;

}

Rox_ErrorCode rox_image_rgba_save_ppm ( const char * path, Rox_Image_RGBA in )
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Uchar * rgb = NULL;
    FILE * file = NULL;

    if (!path || !in )
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    Rox_Sint cols = 0, rows = 0;
    error = rox_image_rgba_get_size(&rows, &cols, in);
    ROX_ERROR_CHECK_TERMINATE ( error );

    file = fopen(path, "wb");
    if (!file)
    { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_ppm_save_header(file, rows, cols);
    ROX_ERROR_CHECK_TERMINATE ( error );

    rgb = (Rox_Uchar*) rox_memory_allocate ( 3*rows*cols, 1 );
    if(rgb == 0) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    error = rox_roxrgba_to_rgb ( rgb, in );
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_ppm_write_content ( file, rgb, cols, rows );
    ROX_ERROR_CHECK_TERMINATE ( error );

 function_terminate:
    if (file)
       fclose(file);
    if (rgb)
       rox_memory_delete(rgb);
   return error;
}
