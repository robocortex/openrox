//==============================================================================
//
//    OPENROX   : File image_warp_matsl3.h
//
//    Contents  : API of image warping module with matrix in SL3
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image_roi.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_new_copy_roi ( 
   Rox_Image * dest, 
   const Rox_Rect_Sint roi, 
   const Rox_Image source 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !dest )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !source || !roi )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_image_new ( dest, roi->width, roi->height );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dest_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dest_data, *dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** source_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &source_data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint v = 0; v < roi->height; v++ )
   {
      for ( Rox_Sint u = 0; u < roi->width ; u++ )
      {
         dest_data[v][u] =  source_data[v+roi->y][u+roi->x];
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_image_new_roi ( 
   Rox_Image * image_ref, 
   Rox_Imask * imask_ref,
   Rox_Rect_Sint_Struct * roi,
   const Rox_Image image,
   const Rox_Imask imask 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if ( !image_ref || !imask_ref || !roi ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image || !imask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_imask_get_roi ( roi, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_new_copy_roi ( image_ref, roi, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_new_copy_roi ( imask_ref, roi, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
