//==============================================================================
//
//    OPENROX   : File image_window_difference_score.c
//
//    Contents  : Implementation of image window difference score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image_window_difference_score.h"

#include <generated/dynvec_rect_sint_struct.h>
#include <generated/dynvec_double_struct.h>
#include <generated/objset_imask_struct.h>
#include <generated/array2d_double.h>

#include <baseproc/array/crosscor/zncrosscor.h>

Rox_ErrorCode rox_image_window_nomask_difference_get_score ( 
   Rox_Double * score, 
   const Rox_Rect_Sint window, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image patch_ref = NULL;
   Rox_Image patch_cur = NULL;

   if ( !image_ref || !image_cur || !score || ! window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create sub images based on window data
   error = rox_array2d_uchar_new_subarray2d( &patch_ref, image_ref, window->y, window->x, window->height, window->width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new_subarray2d( &patch_cur, image_cur, window->y, window->x, window->height, window->width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute score
   error = rox_array2d_uchar_zncc_nomask_normalizedscore ( score, patch_ref, patch_cur );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_uchar_del(&patch_ref);
   rox_array2d_uchar_del(&patch_cur);

   return error;
}

Rox_ErrorCode rox_image_windows_nomask_difference_get_scores ( 
   Rox_DynVec_Double score_list, 
   const Rox_DynVec_Rect_Sint window_list, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !image_ref || !image_cur || !score_list || ! window_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint k=0; k<window_list->used; k++)
   {
      error = rox_image_window_nomask_difference_get_score ( &score_list->data[k], &window_list->data[k], image_ref, image_cur );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_window_difference_get_score ( 
   Rox_Double * score, 
   Rox_Rect_Sint window, 
   Rox_Imask imask, 
   Rox_Image image_ref, 
   Rox_Image image_cur 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image patch_ref = NULL;
   Rox_Image patch_cur = NULL;

   if ( !image_ref || !image_cur || !score || ! window) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create sub images based on window data
   error = rox_array2d_uchar_new_subarray2d ( &patch_ref, image_ref, window->y, window->x, window->height, window->width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new_subarray2d ( &patch_cur, image_cur, window->y, window->x, window->height, window->width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute score
   error = rox_array2d_uchar_zncc_normalizedscore ( score, patch_cur, patch_ref, imask );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_uchar_del(&patch_ref);
   rox_array2d_uchar_del(&patch_cur);

   return error;
}

Rox_ErrorCode rox_image_windows_difference_get_scores ( 
   Rox_DynVec_Double score_list, 
   const Rox_DynVec_Rect_Sint window_list, 
   const Rox_ObjSet_Imask imask_list, 
   const Rox_Image image_ref, 
   const Rox_Image image_cur 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !image_ref || !image_cur || !score_list || ! window_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint k=0; k<window_list->used; k++)
   {
      error = rox_image_window_difference_get_score ( &score_list->data[k], &window_list->data[k], imask_list->data[k], image_ref, image_cur );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
