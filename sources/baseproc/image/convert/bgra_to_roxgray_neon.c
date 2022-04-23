//==============================================================================
//
//    OPENROX   : File bgra_to_roxgray_neon.c
//
//    Contents  : Implementation of bgra_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bgra_to_roxgray.h"
#include <inout/system/errors_print.h

int rox_ansi_bgra_to_roxgray_neon (
   unsigned_char ** output_data, 
   const unsigned_char  * buffer_data, 
   const int rows,
   const int cols,
   const int stride
)
{
   // warning stride is unused : compare to non neon function to see if it is unnecessary in that function too
   
   // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
   // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
   // [ 0.212671, 0.715160, 0.072169 ] which can be approximated by the follwing integer values:
   //  54 / 256 = 0.210938, 183 / 256 = 0.714844, 18 / 256 = 0.070312

   uint8x8_t rfac = vdup_n_u8 ( 54);
   uint8x8_t gfac = vdup_n_u8 (183);
   uint8x8_t bfac = vdup_n_u8 ( 18);
   
   int cols8 = cols / 8;
   
   for ( int i = 0; i < rows; i++ )
   {
      unsigned_char * orow = output_data[i];
      
      for ( int j = 0; j < cols8; j++ )
      {
         uint16x8_t  temp;
         uint8x8x4_t rgb  = vld4_u8(buffer_data);
         uint8x8_t result;
         
         temp = vmull_u8 (rgb.val[0], bfac);
         temp = vmlal_u8 (temp, rgb.val[1], gfac);
         temp = vmlal_u8 (temp, rgb.val[2], rfac);
         
         result = vshrn_n_u16(temp, 8);
         vst1_u8(orow, result);
         
         orow+=8;
         buffer_data+=32;
      }
   }
   
   return 0;
}


Rox_ErrorCode rox_bgra_to_roxgray (
   Rox_Image output, 
   const Rox_Uchar * buffer_data, 
   const Rox_Sint stride
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

#ifdef integrated 
   // warning stride is unused : compare to non neon function to see if it is unnecessary in that function too
   
   // The ITU-R BT.709 standard used for HDTV developed by the ATSC and the sRGB
   // standard (https://en.wikipedia.org/wiki/SRGB) use the color coefficients
   // [ 0.212671, 0.715160, 0.072169 ] which can be approximated by the follwing integer values:
   //  54 / 256 = 0.210938, 183 / 256 = 0.714844, 18 / 256 = 0.070312

   uint8x8_t rfac = vdup_n_u8 ( 54);
   uint8x8_t gfac = vdup_n_u8 (183);
   uint8x8_t bfac = vdup_n_u8 ( 18);
#endif
   
   if (!output || !buffer_data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0 , rows = 0;
   error = rox_image_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** output_data = NULL;
   error = rox_image_get_data_pointer_to_pointer( &output_data, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef integrated 

   Rox_Sint cols8 = cols / 8;
   
   for ( Rox_Sint i = 0; i < rows; i++ )
   {
      Rox_Uchar * orow = output_data[i];
      
      for ( Rox_Sint j = 0; j < cols8; j++ )
      {
         uint16x8_t  temp;
         uint8x8x4_t rgb  = vld4_u8(buffer_data);
         uint8x8_t result;
         
         temp = vmull_u8 (rgb.val[0], bfac);
         temp = vmlal_u8 (temp, rgb.val[1], gfac);
         temp = vmlal_u8 (temp, rgb.val[2], rfac);
         
         result = vshrn_n_u16(temp, 8);
         vst1_u8(orow, result);
         
         orow+=8;
         buffer_data+=32;
      }
   }
#else
   error = rox_ansi_bgra_to_roxgray_neon ( output_data, buffer_data, rows, cols, stride );
#endif

function_terminate:
   return error;
}
