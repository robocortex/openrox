//==============================================================================
//
//    OPENROX   : File image.c
//
//    Contents  : Implementation of image module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image.h"

#include <string.h>
#include <math.h>

#include <system/memory/array2d_struct.h>

#include <baseproc/array/flip/flipud.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/substract/substract.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>
#include <baseproc/image/convert/gray_to_roxgray.h>
#include <baseproc/image/convert/roxgray_to_gray.h>
#include <baseproc/image/convert/yuv422_to_roxgray.h>
#include <baseproc/image/convert/rgba_to_roxgray.h>
#include <baseproc/image/convert/bgra_to_roxgray.h>
#include <baseproc/image/convert/argb_to_roxgray.h>
#include <baseproc/image/convert/rgb_to_roxgray.h>
#include <baseproc/image/convert/bgr_to_roxgray.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>
#include <baseproc/image/convert/alpha8_to_roxgray.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_new ( Rox_Image * image, const Rox_Sint cols, const Rox_Sint rows )
{
   return rox_array2d_uchar_new ( image, rows, cols );
}

Rox_ErrorCode rox_image_del ( Rox_Image * image )
{
   return rox_array2d_uchar_del(image);
}

Rox_ErrorCode rox_image_new_read_pgm ( Rox_Image *image, const char *filename )
{
    return rox_array2d_uchar_new_pgm ( image, filename );
}

Rox_ErrorCode rox_image_read_pgm ( Rox_Image image, const char *filename )
{
    return rox_array2d_uchar_read_pgm(image, filename);
}

Rox_ErrorCode rox_image_new_read_ppm ( Rox_Image *image, const char *filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Image_RGBA image_rgba = NULL;
   error = rox_image_rgba_new_read_ppm ( &image_rgba, filename );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_rgba_get_size ( &rows, &cols, image_rgba );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_new ( image, cols, rows );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxrgba_to_roxgray ( *image, image_rgba );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_image_rgba_del ( &image_rgba );

   return error;
}

Rox_ErrorCode rox_image_read_ppm ( Rox_Image image, const char *filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image_RGBA image_rgba = NULL;

   error = rox_image_rgba_new_read_ppm ( &image_rgba, filename );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxrgba_to_roxgray ( image, image_rgba );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_image_rgba_del(&image_rgba);

   return error;
}

Rox_ErrorCode rox_image_save_pgm ( const char * filename, const Rox_Image image )
{
    return rox_array2d_uchar_save_pgm ( filename, image );
}

Rox_ErrorCode rox_image_get_cols ( Rox_Sint * cols, const Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cols)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_cols(cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

 function_terminate:
  return error;
}

Rox_ErrorCode rox_image_get_rows ( Rox_Sint * rows, const Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rows)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_rows(rows, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_get_size(Rox_Sint * rows, Rox_Sint * cols, const Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rows || !cols)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(rows, cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_match_size ( const Rox_Image image_1, const Rox_Image image_2 )
{
   return rox_array2d_uchar_match_size ( image_1, image_2 );
}

Rox_ErrorCode rox_image_check_size ( const Rox_Image image, const Rox_Sint rows, const Rox_Sint cols)
{
   return rox_array2d_uchar_check_size ( image, rows, cols );
}

Rox_ErrorCode rox_image_get_bytesperrow ( Rox_Sint * bytesPerRow, const Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!bytesPerRow)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride ( &stride, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   * bytesPerRow = (Rox_Sint) stride;

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_get_data_pointer_to_pointer ( Rox_Uchar *** rowsptr, const Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rowsptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_data_pointer_to_pointer(rowsptr, image);

 function_terminate:
  return error;
}

Rox_ErrorCode rox_image_set_data (
   Rox_Image image,
   const Rox_Uchar * data,
   const Rox_Sint bytesPerRow,
   const enum Rox_Image_Format format
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image || !data)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch(format)
   {
      default:

         error = ROX_ERROR_INVALID_VALUE;
         ROX_ERROR_CHECK_TERMINATE ( error );

      case  Rox_Image_Format_Grays:

         error = rox_gray_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_YUV422:

         error = rox_yuv422_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_RGBA:
         error = rox_rgba_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_BGRA:
         error = rox_bgra_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_ARGB:
         error = rox_argb_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_BGR:
         error = rox_bgr_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_RGB:
         error = rox_rgb_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_RGBA_FlippedUpsideDown:
         // To be replaced by a dedicated function rgba_flip_to_roxgray ?
         error = rox_rgba_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_image_flip(image);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_Alpha8_32bits:
         error = rox_alpha8_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;

      case Rox_Image_Format_Alpha8_32bits_FlippedUpsideDown:
         // To be replaced by a dedicated function alpha8_flip_to_roxgray ?
         error = rox_alpha8_to_roxgray(image, data, bytesPerRow);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_image_flip(image);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
   }

function_terminate:
    return error;
}

Rox_ErrorCode rox_image_get_data ( Rox_Uchar * data, const Rox_Image image )
{
   return rox_roxgray_to_gray ( data, image );
}

Rox_ErrorCode rox_image_copy(Rox_Image dest, const  Rox_Image source)
{
   return rox_array2d_uchar_copy(dest, source);
}

Rox_ErrorCode rox_image_copy_flip(Rox_Image dest, const Rox_Image source)
{
   return rox_array2d_uchar_copy_flip_ud(dest, source);
}

Rox_ErrorCode rox_image_flip(Rox_Image image_inout)
{
   return rox_array2d_uchar_flip_ud(image_inout);
}

Rox_ErrorCode rox_image_new_copy ( Rox_Image * image_out, Rox_Image image_inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_inp || !image_out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, image_inp);

   error = rox_image_new(image_out, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_copy(*image_out, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_new_square_centered ( Rox_Image * image_out, const Rox_Image image_inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_inp || !image_out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows > cols)
   {
      error = rox_image_new(image_out, rows, rows);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uchar_fillval(*image_out, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

	  // Add explicit cast in floor function since it may be overloaded
	  Rox_Sint col_ini = (Rox_Sint) floor((double)(rows - cols) / 2);
      //Rox_Sint col_end = col_ini + cols - 1;

      Rox_Uchar ** rows_ptr_inp = NULL;
      error = rox_image_get_data_pointer_to_pointer ( &rows_ptr_inp,  image_inp );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Uchar ** rows_ptr_out = NULL;
      error = rox_image_get_data_pointer_to_pointer ( &rows_ptr_out, *image_out );
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Sint i = 0; i < rows; i++)
      {
         Rox_Uchar * ptr = rows_ptr_out[i]+col_ini;

         memcpy(ptr, rows_ptr_inp[i], cols);
      }
   }
   else if (rows < cols)
   {
      error = rox_image_new(image_out, cols, cols);
      ROX_ERROR_CHECK_TERMINATE ( error );

	   // Add explicit cast in floor function since it may be overloaded
	   Rox_Sint row_ini = (Rox_Sint) floor((double) (cols - rows) / 2);
      Rox_Sint row_end = row_ini + rows;

      Rox_Uchar ** rows_ptr_inp = NULL;
      error = rox_array2d_uchar_get_data_pointer_to_pointer( &rows_ptr_inp,  image_inp );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Uchar ** rows_ptr_out = NULL;
      error = rox_array2d_uchar_get_data_pointer_to_pointer( &rows_ptr_out, *image_out );
      ROX_ERROR_CHECK_TERMINATE ( error );

      for ( Rox_Sint i = row_ini; i < row_end; i++ )
      {
         memcpy(rows_ptr_out[i], rows_ptr_inp[i-row_ini], cols);
      }
   }
   else
   {
      error = rox_image_new(image_out, cols, rows);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_image_copy(*image_out, image_inp);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_convert_uchar_to_uchar_nostride ( unsigned char * data_uchar, Rox_Image image )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint count = 0;
   Rox_Sint cols = 0, rows = 0;

   Rox_Uchar ** image_data = NULL;
   error = rox_image_get_data_pointer_to_pointer ( &image_data, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size ( &rows, &cols, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint v = 0; v < rows; v++) 
   {
      for (Rox_Sint u = 0; u < cols; u++) 
      {
         data_uchar[count] = image_data[v][u]; 
         count++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_convert_uchar_to_float_nostride ( float * data_float, Rox_Image uchar )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint count = 0;
   Rox_Sint cols = 0, rows = 0;

   Rox_Uchar ** data_uchar = NULL;
   error = rox_image_get_data_pointer_to_pointer ( &data_uchar, uchar );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size ( &rows, &cols, uchar );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint v = 0; v < rows; v++) 
   {
      for (Rox_Sint u = 0; u < cols; u++) 
      {
         data_float[count] = (float) data_uchar[v][u]; 
         count++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_abs_difference(Rox_Image dest, const  Rox_Image source_1, const  Rox_Image source_2)
{
   return rox_array2d_uchar_substract(dest, source_1, source_2);
}
