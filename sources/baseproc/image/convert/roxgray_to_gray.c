//==============================================================================
//
//    OPENROX   : File roxgray_to_gray.c
//
//    Contents  : Implementation of roxgray_to_gray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "roxgray_to_gray.h"
#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_roxgray_to_gray ( Rox_Uchar * dest, const Rox_Image input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !dest || !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size ( &rows, &cols, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** pixels = NULL;
   error = rox_image_get_data_pointer_to_pointer ( &pixels, input );
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for (Rox_Sint i = 0; i < rows; i++)
   {
      memcpy ( dest + i * cols, pixels[i], cols );
   }

function_terminate:
   return error;
}
