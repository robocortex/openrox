//==============================================================================
//
//    OPENROX   : File argb_to_roxrgba.c
//
//    Contents  : Implementation of argb_to_roxrgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "argb_to_roxrgba.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_argb_to_roxrgba(Rox_Image_RGBA dest, const Rox_Uchar * buffer, const Rox_Sint stride)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!buffer || !dest)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = (Rox_Uchar*)dd[i];
      const Rox_Uchar * rs = buffer + i * stride;

      for (Rox_Sint j = 0; j < width; j++)
      {
         rd[0] = rs[1];
         rd[1] = rs[2];
         rd[2] = rs[3];
         rd[3] = rs[0];
         rd += 4;
         rs += 4;
      }
   }

function_terminate:
   return error;
}


