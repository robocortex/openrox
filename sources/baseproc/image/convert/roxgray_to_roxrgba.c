//==============================================================================
//
//    OPENROX   : File roxgray_to_roxrgba.c
//
//    Contents  : Implementation of roxgray_to_roxrgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "roxgray_to_roxrgba.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_roxgray_to_roxrgba (
   Rox_Image_RGBA image_gray, 
   const Rox_Image image_rgba
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_rgba || !image_gray) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_check_size(image_rgba, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint  ** dd = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer( &dd, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar ** ds = NULL;
   error = rox_image_get_data_pointer_to_pointer( &ds, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = (Rox_Uchar *) dd[i];
      Rox_Uchar * rs = ds[i];

      for (Rox_Sint j = 0; j < width; j++)
      {
         rd[0] = rs[j];
         rd[1] = rs[j];
         rd[2] = rs[j];
         rd[3] = 255;
         rd += 4;
      }
   }

function_terminate:
   return error;
}
