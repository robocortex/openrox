//==============================================================================
//
//    OPENROX   : File gray_to_roxgray.c
//
//    Contents  : Implementation of gray_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "gray_to_roxgray.h"
#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_gray_to_roxgray(Rox_Image dest, const Rox_Uchar * src, const Rox_Sint stride)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!dest || !src) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar **dd = NULL;
   error = rox_image_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for(Rox_Sint i = 0; i < height; i++)
   {
      memcpy(dd[i], src + i * stride, width);
   }

function_terminate:
   return error;
}
