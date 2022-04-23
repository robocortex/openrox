//============================================================================
//
//    OPENROX   : File image_rgba.c
//
//    Contents  : Implementation of image_rgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "image_rgba.h"

#include <string.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>

#include <baseproc/image/convert/gray_to_roxrgba.h>
#include <baseproc/image/convert/yuv422_to_roxrgba.h>
#include <baseproc/image/convert/rgba_to_roxrgba.h>
#include <baseproc/image/convert/bgra_to_roxrgba.h>
#include <baseproc/image/convert/argb_to_roxrgba.h>
#include <baseproc/image/convert/rgb_to_roxrgba.h>
#include <baseproc/image/convert/bgr_to_roxrgba.h>
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <baseproc/array/flip/flipud.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_new(Rox_Image_RGBA *image, Rox_Sint cols, Rox_Sint rows)
{
   return rox_array2d_uint_new(image, rows, cols);
}

Rox_ErrorCode rox_image_rgba_del(Rox_Image_RGBA *image)
{
   return rox_array2d_uint_del(image);
}

Rox_ErrorCode rox_image_rgba_new_read_pgm(Rox_Image_RGBA *image, const char *filename)
{
   return rox_array2d_uint_rgba_new_pgm(image, filename);
}

Rox_ErrorCode rox_image_rgba_read_pgm(Rox_Image_RGBA image, const char *filename)
{
   return rox_array2d_uint_rgba_read_pgm(image, filename);
}

Rox_ErrorCode rox_image_rgba_save_pgm(const char *filename, Rox_Image_RGBA image)
{
   return rox_array2d_uint_rgba_save_pgm(filename, image);
}

Rox_ErrorCode rox_image_rgba_copy(Rox_Image_RGBA image_rgba_out, Rox_Image_RGBA image_rgba_inp)
{
   return rox_array2d_uint_copy(image_rgba_out, image_rgba_inp);
}

Rox_ErrorCode rox_image_rgba_get_cols(Rox_Sint * cols, Rox_Image_RGBA image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!cols) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_cols(cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_get_rows(Rox_Sint *rows, Rox_Image_RGBA image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!rows) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_rows(rows, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_get_size(Rox_Sint * rows, Rox_Sint * cols, Rox_Image_RGBA image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!rows || !cols || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_rows(rows, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_cols(cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_get_bytesperrow(Rox_Sint * bytesPerRow, Rox_Image_RGBA image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!bytesPerRow) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_stride(bytesPerRow, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_get_data_pointer_to_pointer(Rox_Uint *** rowsptr, Rox_Image_RGBA image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_data_pointer_to_pointer ( rowsptr, image );

 function_terminate:
  return error;
}

Rox_ErrorCode rox_image_rgba_get_data_pointer ( Rox_Uint ** rowsptr, Rox_Image_RGBA image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_get_data_pointer(rowsptr, image);

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_set_data(Rox_Image_RGBA image, Rox_Uchar *data, Rox_Sint bytesPerRow, enum Rox_Image_Format format)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!image || !data)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    switch(format)
    {
        case Rox_Image_Format_Grays:
            error = rox_gray_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_YUV422:
            error = rox_yuv422_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_RGBA:
            error = rox_rgba_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_BGRA:
            error = rox_bgra_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_ARGB:
            error = rox_argb_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_BGR:
            error = rox_bgr_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_RGB:
            error = rox_rgb_to_roxrgba(image, data, bytesPerRow); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            break;

        case Rox_Image_Format_RGBA_FlippedUpsideDown:
           // To be replaced by a dedicated function rgba_flip_to_roxrba ?
           error = rox_rgba_to_roxrgba(image, data, bytesPerRow);
           error = rox_image_rgba_flip(image);
           break;

        default:
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
            break;

    }

function_terminate:
    return error;
}

Rox_ErrorCode rox_image_rgba_flip(Rox_Image_RGBA image_inout)
{
   return rox_array2d_uint_flip_ud(image_inout);
}

Rox_ErrorCode rox_image_rgba_inlay(Rox_Image_RGBA image_out, Rox_Image_RGBA image_inp, Rox_Sint u, Rox_Sint v)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    Rox_Array2D_Uint image_sub = NULL;

    if (!image_out || !image_inp)
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    Rox_Sint rows = 0, cols = 0;
    error = rox_array2d_uint_get_size(&rows, &cols, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_uint_new_subarray2d(&image_sub, image_out, v, u, rows, cols);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_uint_copy(image_sub, image_inp);
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    rox_array2d_uint_del(&image_sub);
    return error;
}

Rox_ErrorCode rox_image_rgba_convert_image(Rox_Image_RGBA image_rgba_out, Rox_Image image_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_roxgray_to_roxrgba ( image_rgba_out, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_image_rgba_new_convert_image ( Rox_Image_RGBA * image_rgba_out, Rox_Image image_inp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint cols=0, rows =0;

   error = rox_image_get_size (&rows, &cols, image_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_rgba_new ( image_rgba_out, cols, rows );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxgray_to_roxrgba ( *image_rgba_out, image_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_image_convert_uint_to_uint_nostride ( unsigned int * data_uint, Rox_Image_RGBA image_rgba )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint count = 0;
   Rox_Sint cols = 0, rows = 0;

   Rox_Uint ** image_rgba_data = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer ( &image_rgba_data, image_rgba );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_rgba_get_size ( &rows, &cols, image_rgba );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint v = 0; v < rows; v++) 
   {
      for (Rox_Sint u = 0; u < cols; u++) 
      {
         data_uint[count] = image_rgba_data[v][u]; 
         count++;
      }
   }

function_terminate:
   return error;
}
