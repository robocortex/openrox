//==============================================================================
//
//    OPENROX   : File yuv422_to_roxgray.c
//
//    Contents  : Implementation of yuv422_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "yuv422_to_roxgray.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_yuv422_to_roxgray ( Rox_Image dest, const Rox_Uchar * src, const Rox_Uint stride)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest || !src) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (stride < (Rox_Uint) (width * 2)) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** ddst = NULL;
   error = rox_image_get_data_pointer_to_pointer( &ddst, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint halfwidth = width / 2;

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = ddst[i];
      const Rox_Uchar * rs = src + i * stride;

      for (Rox_Uint j = 0; j < halfwidth; j++)
      {
         Rox_Sint y1 = rs[0];
         Rox_Sint y2 = rs[2];

         rd[0] = y1;
         rd[1] = y2;

         rs += 4;
         rd += 2;
      }
   }

function_terminate:
   return error;
}

