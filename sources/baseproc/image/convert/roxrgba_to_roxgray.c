//==============================================================================
//
//    OPENROX   : File roxrgba_to_roxgray.c
//
//    Contents  : Implementation of roxrgba_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "roxrgba_to_roxgray.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_roxrgba_to_roxgray ( Rox_Image image_gray, const Rox_Image_RGBA image_rgba)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_rgba || !image_gray) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_check_size(image_gray, height, width); 
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Uint ** ds = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer( &ds, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Uchar ** dd = NULL;
   error = rox_image_get_data_pointer_to_pointer( &dd, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = dd[i];
      Rox_Uchar * rs = (Rox_Uchar*) ds[i];

      for ( Rox_Sint j = 0; j < width; j++)
      {
         Rox_Uint r = rs[0];
         Rox_Uint g = rs[1];
         Rox_Uint b = rs[2];

         // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
         // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
         // [ 0.212671, 0.715160, 0.072169 ] computing the luma component as follows:
         rd[j] = (Rox_Uchar) ((r * 0.212671f) + (g * 0.715160f) + (b * 0.072169f));

         rs += 4;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_roxrgba_to_roxgray_approx ( Rox_Image image_gray, const Rox_Image_RGBA image_rgba )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_rgba || !image_gray)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_check_size(image_gray, height, width);
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Uint ** data_rgba = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer(&data_rgba, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Uchar ** data_gray = NULL;
   error = rox_image_get_data_pointer_to_pointer(&data_gray, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = data_gray[i];
      Rox_Uchar * rs = (Rox_Uchar*) data_rgba[i];

      for ( Rox_Sint j = 0; j < width; j++)
      {
         // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
         // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
         // [ 0.212671, 0.715160, 0.072169 ] which can be approximated by the follwing integer values:
         //  54 / 256 = 0.210938, 183 / 256 = 0.714844, 18 / 256 = 0.070312

         Rox_Uint r =  54 * rs[0];
         Rox_Uint g = 183 * rs[1];
         Rox_Uint b =  18 * rs[2];

         rd[j] = (Rox_Uchar) ((r + g + b) >> 8); // Divide (r+g+b) by 256 by shift 8 bits

         rs += 4;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_roxrgba_flip_to_roxgray_approx(Rox_Image image_gray, const Rox_Image_RGBA image_rgba)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_rgba || !image_gray)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_check_size(image_gray, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data_rgba = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer(&data_rgba, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** data_gray = NULL;
   error = rox_image_get_data_pointer_to_pointer(&data_gray, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint last_row = height - 1;

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = data_gray[last_row-i];
      Rox_Uchar * rs = (Rox_Uchar*) data_rgba[i];

      for ( Rox_Sint j = 0; j < width; j++)
      {
         // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
         // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
         // [ 0.212671, 0.715160, 0.072169 ] which can be approximated by the follwing integer values:
         //  54 / 256 = 0.210938, 183 / 256 = 0.714844, 18 / 256 = 0.070312
         Rox_Uint r =  54 * rs[0];
         Rox_Uint g = 183 * rs[1];
         Rox_Uint b =  18 * rs[2];

         rd[j] = (Rox_Uchar) ((r + g + b) >> 8); // Divide (r+g+b) by 256 by shift 8 bits

         rs += 4;
      }
   }

function_terminate:
   return error;
}

