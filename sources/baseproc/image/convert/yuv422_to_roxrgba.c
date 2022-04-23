//==============================================================================
//
//    OPENROX   : File yuv422_to_roxrgba.c
//
//    Contents  : Implementation of yuv422_to_roxrgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "yuv422_to_roxrgba.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_yuv422_to_roxrgba ( Rox_Image_RGBA dest, const Rox_Uchar * src, const Rox_Uint stride)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint y1,u,y2,v,c,d,e;
   Rox_Sint r,g,b;

   if (!dest || !src)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the size of the destination buffer (rgba)
   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (stride < (Rox_Uint) (width * 2)) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //
   Rox_Uint **ddst = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer( &ddst, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint halfwidth = width / 2;

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = (Rox_Uchar*) ddst[i];
      const Rox_Uchar * rs = src + i * stride;

      for (Rox_Uint j = 0; j < halfwidth; j++)
      {
         y1 = rs[0];
         u = rs[1];

         y2 = rs[2];
         v = rs[3];

         // Convert yuv444 to rgba8888
         // create a function convert_yuv444_to_rgba8888(Rox_Uchar * rd, Rox_Uchar y, Rox_Uchar u, Rox_Uchar v)
         c = y1 - 16;
         d = u - 128;
         e = v - 128;

         r = ((298 * c + 409 * e + 128) >> 8);
         g = ((298 * c - 100 * d - 208 * e + 128) >> 8);
         b = ((298 * c + 516 * d + 128) >> 8);

         r = r>255? 255 : r<0 ? 0 : r;
         g = g>255? 255 : g<0 ? 0 : g;
         b = b>255? 255 : b<0 ? 0 : b;

         // Convert yuv444 to rgba8888
         // create a function convert_yuv444_to_rgba8888(Rox_Uchar * rd, Rox_Uchar y, Rox_Uchar u, Rox_Uchar v)
         rd[0] = r;
         rd[1] = g;
         rd[2] = b;
         rd[3] = 255;

         // Convert yuv444 to rgba8888
         c = y2 - 16;
         d = u - 128;
         e = v - 128;
         r = ((298 * c + 409 * e + 128) >> 8);
         g = ((298 * c - 100 * d - 208 * e + 128) >> 8);
         b = ((298 * c + 516 * d + 128) >> 8);
         r = r>255? 255 : r<0 ? 0 : r;
         g = g>255? 255 : g<0 ? 0 : g;
         b = b>255? 255 : b<0 ? 0 : b;

         rd[4] = r;
         rd[5] = g;
         rd[6] = b;
         rd[7] = 255;

         // Increment pointer of source buffer
         rs += 4;
         // Increment pointer of destination buffer
         rd += 8;
      }
   }

function_terminate:
   return error;
}
