//==============================================================================
//
//    OPENROX   : File alpha8_to_roxgray.c
//
//    Contents  : Implementation of alpha8_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "alpha8_to_roxgray.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_alpha8_to_roxgray ( Rox_Image dest, const Rox_Uchar * buffer, const Rox_Sint stride )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar * rd_out = NULL;

   if ( !buffer || !dest ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_get_size(&height, &width, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** d_out = NULL;
   error = rox_image_get_data_pointer_to_pointer( &d_out, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i_in = 0; i_in < height; i_in++)
   {
      Rox_Sint i_out = i_in;

      rd_out = d_out[i_out];
      const unsigned char* rd_in = buffer + i_in * stride;

      for (Rox_Sint j = 0; j < width; j++)
      {
         rd_out[j] = rd_in[3];

         rd_in += 4;
      }
   }

function_terminate:
   return error;
}
