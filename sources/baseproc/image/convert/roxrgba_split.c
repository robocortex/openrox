//==============================================================================
//
//    OPENROX   : File roxrgba_split.c
//
//    Contents  : Implementation of roxrgba_split module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "roxrgba_split.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_roxrgba_split (
   Rox_Image image_red, 
   Rox_Image image_green, 
   Rox_Image image_blue, 
   const Rox_Image_RGBA image_rgba
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_rgba || !image_red || !image_green || !image_blue) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_image_rgba_get_size(&height, &width, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_check_size(image_red, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_check_size(image_green, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_check_size(image_blue, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint  ** ds  = NULL;
   error = rox_image_rgba_get_data_pointer_to_pointer( &ds, image_rgba);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Uchar ** ddr = NULL;
   error = rox_image_get_data_pointer_to_pointer( &ddr, image_red);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Uchar ** ddg = NULL;
   error = rox_image_get_data_pointer_to_pointer( &ddg, image_green);
   ROX_ERROR_CHECK_TERMINATE ( error ); 
   
   Rox_Uchar ** ddb = NULL;
   error = rox_image_get_data_pointer_to_pointer( &ddb, image_blue);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Uchar * rdr = ddr[i];
      Rox_Uchar * rdg = ddg[i];
      Rox_Uchar * rdb = ddb[i];
      Rox_Uchar * rs = (Rox_Uchar*)ds[i];

      for (Rox_Sint j = 0; j < width; j++)
      {
         Rox_Uint r = rs[0];
         Rox_Uint g = rs[1];
         Rox_Uint b = rs[2];

         rdr[j] = r;
         rdg[j] = g;
         rdb[j] = b;

         rs += 4;
      }
   }

function_terminate:
   return error;
}
