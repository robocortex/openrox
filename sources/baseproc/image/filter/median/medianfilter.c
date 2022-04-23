//==============================================================================
//
//    OPENROX   : File medianfilter.c
//
//    Contents  : Implementation of medianfilter module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "medianfilter.h"
#include <inout/system/errors_print.h>
#include <baseproc/image/image.h>

Rox_ErrorCode rox_image_filter_median(Rox_Image dest, Rox_Image source, Rox_Sint radius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint hist[256]; // 256 = 2^8 possible values for an uchar from 0 to 255
   const Rox_Sint diameter = radius * 2 + 1;
   const Rox_Sint square = diameter * diameter;
   const Rox_Sint limit = square / 2;

   if (!dest || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (radius == 0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(source, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint r = radius;

   for (Rox_Sint i = radius; i < rows - radius; i++)
   {
      for (Rox_Sint j = radius; j < cols - radius; j++)
      {
         for (Rox_Sint m = 0; m < 256; m++)
         {
            hist[m] = 0;
         }

         for (Rox_Sint k = -r; k <= r; k++)
         {
            for (Rox_Sint l = -r; l <= r; l++)
            {
               hist[ds[i+k][j+l]]++;
            }
         }

         for (Rox_Sint m = 1; m < 256; m++)
         {
            hist[m] = hist[m - 1] + hist[m];
            if (hist[m] >(Rox_Uint) limit)
            {
               dd[i][j] = m;

               break;
            }
         }
      }
   }

function_terminate:
   return error;
}