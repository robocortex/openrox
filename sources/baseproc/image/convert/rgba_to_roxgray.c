//==============================================================================
//
//    OPENROX   : File rgba_to_roxgray.c
//
//    Contents  : Implementation of rgba_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "rgba_to_roxgray.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_rgba_to_roxgray_approx ( Rox_Image dest, const Rox_Uchar * buffer, const Rox_Sint stride )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !buffer || !dest ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dd = NULL;
   error = rox_image_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = dd[i];
      const Rox_Uchar * rs = buffer + i * stride;

      for (Rox_Sint j = 0; j < width; j++)
      {
         // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
         // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
         // [ 0.212671, 0.715160, 0.072169 ] which can be approximated by the following integer values:
         //  54 / 256 = 0.210938, 183 / 256 = 0.714844, 18 / 256 = 0.070312
         
         Rox_Sint r =  54 * rs[0];
         Rox_Sint g = 183 * rs[1];
         Rox_Sint b =  18 * rs[2];

         // Rox_Sint r =  77 * rs[0];
         // Rox_Sint g = 151 * rs[1];
         // Rox_Sint b =  28 * rs[2];
         
         rd[j] = (Rox_Uchar) ((r + g + b) >> 8);

         rs += 4;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_rgba_to_roxgray(Rox_Array2D_Uchar dest, const Rox_Uchar * buffer, const Rox_Sint stride)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!buffer || !dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar **dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = dd[i];
      const Rox_Uchar * rs = buffer + i * stride;

      for (Rox_Sint j = 0; j < width; j++)
      {
         Rox_Uchar r = rs[0];
         Rox_Uchar g = rs[1];
         Rox_Uchar b = rs[2];

         // The ITU-R BT.709 standard used for HDTV developed by the ATSC uses the color coefficients
         // [ 0.212671, 0.715160, 0.072169 ] computing the luma component as follows:
         rd[j] = (Rox_Uchar) ((r * 0.212671f) + (g * 0.715160f) + (b * 0.072169f));

         rs += 4;
      }
   }

function_terminate:
   return error;
}
