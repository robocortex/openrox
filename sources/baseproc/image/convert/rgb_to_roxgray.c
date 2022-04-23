//==============================================================================
//
//    OPENROX   : File rgb_to_roxgray.c
//
//    Contents  : Implementation of rgb_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "rgb_to_roxgray.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_rgb_to_roxgray(Rox_Array2D_Uchar dest, const Rox_Uchar * buffer, const Rox_Sint stride)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!buffer || !dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = dd[i];
      Rox_Uchar * rs = (Rox_Uchar *) buffer + i * stride;

      for (Rox_Sint j = 0; j < width; j++)
      {
         Rox_Uchar r = rs[0];
         Rox_Uchar g = rs[1];
         Rox_Uchar b = rs[2];

         // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
         // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
         // [ 0.212671, 0.715160, 0.072169 ] computing the luma component as follows:
         rd[j] = (Rox_Uchar) ((r * 0.212671f) + (g * 0.715160f) + (b * 0.072169f));

         rs += 3;
      }
   }

function_terminate:
   return error;
}

