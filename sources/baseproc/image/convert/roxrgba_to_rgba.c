//==============================================================================
//
//    OPENROX   : File roxrgba_to_rgba.c
//
//    Contents  : Implementation of roxrgba_to_rgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "roxrgba_to_rgba.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_roxrgba_to_rgba(Rox_Uchar * buffer, const Rox_Array2D_Uint source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!buffer || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uint_get_size(&height, &width, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dd, source);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rd = (Rox_Uchar*) dd[i];
      Rox_Uchar * rs = buffer + i * width * 4;

      for ( Rox_Sint j = 0; j < width; j++)
      {
         rs[0] = rd[0];
         rs[1] = rd[1];
         rs[2] = rd[2];
         rs[3] = rd[3];

         rd += 4;
         rs += 4;
      }
   }

function_terminate:
   return error;
}


