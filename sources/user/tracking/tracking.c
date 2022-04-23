//==============================================================================
//
//    OPENROX   : File tracking.c
//
//    Contents  : Implementation of tracking module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking.h"
#include "tracking_params.h"

#include <stdio.h>

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>

#include <inout/numeric/array2d_print.h>
#include <inout/numeric/array2d_save.h>
#include <inout/system/errors_print.h>

#include <user/tracking/tracking_sl3.h>
#include <user/tracking/tracking_tu_tv_s_r.h>
#include <user/tracking/tracking_tu_tv_su_sv.h>

Rox_ErrorCode rox_tracking_new (
  Rox_Tracking *tracking, 
  const Rox_Tracking_Params params, 
  const Rox_Image model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking ret = NULL;
   
   if (!tracking || !params || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *tracking = NULL;

   switch (params->usecase)
   {
       case Rox_Tracking_UseCase_SL3:
       {
          Rox_Tracking_SL3 ret_ = 0;
          error = rox_tracking_sl3_new(&ret_, params, model); 
          ROX_ERROR_CHECK_TERMINATE(error)
          ret = (Rox_Tracking) ret_;
          break;
       }
       case Rox_Tracking_UseCase_tu_tv_s_r:
       {
          Rox_Tracking_tu_tv_s_r ret_ = 0;
          error = rox_tracking_tu_tv_s_r_new(&ret_, params, model); 
          ROX_ERROR_CHECK_TERMINATE(error)
          ret = (Rox_Tracking) ret_;
          break;
       }
       case Rox_Tracking_UseCase_tu_tv_su_sv:
       {
          Rox_Tracking_tu_tv_su_sv ret_ = 0;
          error = rox_tracking_tu_tv_su_sv_new(&ret_, params, model); 
          ROX_ERROR_CHECK_TERMINATE(error)
          ret = (Rox_Tracking) ret_;
          break;
       }
       default:
       {
          error = ROX_ERROR_INVALID_VALUE;
          goto function_terminate;
          break;
       }
   }

   *tracking = ret;

function_terminate:
   if (error) rox_tracking_del(&ret);

   return error;
}

Rox_ErrorCode rox_tracking_free (
  Rox_Tracking tracking
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matsl3_del(&tracking->zoom_homography);
   rox_matsl3_del(&tracking->homography);
   rox_array2d_float_del(&tracking->normalized_ref);
   rox_array2d_float_del(&tracking->normalized_cur);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_del (
  Rox_Tracking * tracking
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Tracking todel = NULL;

   if (!tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *tracking;
   *tracking = NULL;
   
   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!todel->_fptr_del) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // call del function 
   error = todel->_fptr_del(&todel);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_alloc(
  Rox_Tracking tracking, 
  const Rox_Tracking_Params params, 
  const Rox_Image model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint patch_cols, patch_rows;

   if (!tracking || !params || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set to NULL pointers 
   tracking->normalized_ref = NULL;
   tracking->normalized_cur = NULL;
   tracking->zoom_homography = NULL;
   tracking->homography = NULL;
   tracking->_fptr_del = NULL;
   tracking->_fptr_make = NULL;
   tracking->_fptr_set_mask = NULL;

   error = rox_image_get_size ( &patch_rows, &patch_cols, model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &tracking->zoom_homography ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &tracking->homography ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &tracking->normalized_ref, patch_rows, patch_cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_from_uchar_normalize(tracking->normalized_ref, model); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set parameters 
   tracking->score = 0.0;
   tracking->miter = 10;
   tracking->min_score = 0.89;
   tracking->prediction_radius = params->prediction_radius;
   tracking->init_pyr = params->init_pyr;
   tracking->stop_pyr = params->stop_pyr;

function_terminate:
   if (error) rox_tracking_free(tracking);

   return error;
}

Rox_ErrorCode rox_tracking_make (
  Rox_Tracking tracking, 
  const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (tracking == NULL || image == NULL ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Call make function 
   error = tracking->_fptr_make ( tracking, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_get_score (
  Rox_Double *score, 
  const Rox_Tracking tracking
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!score || !tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *score = tracking->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_set_homography (
  Rox_Tracking tracking, 
  const Rox_MatSL3 homography
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!tracking || !homography) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_copy ( tracking->homography, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_set_score_thresh (
  Rox_Tracking tracking, 
  const Rox_Double score_thresh
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   tracking->min_score = score_thresh;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_set_miter (
  Rox_Tracking tracking, 
  const Rox_Sint miter
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (miter == 0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   tracking->miter = miter;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_get_homography (
  Rox_MatSL3 homography, 
  const Rox_Tracking tracking
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!homography || !tracking)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_copy(homography, tracking->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_set_mask (
  Rox_Tracking tracking, 
  const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!mask || !tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Call set_mask function 
   error = tracking->_fptr_set_mask(tracking, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
