//==============================================================================
//
//    OPENROX   : File tracking_tu_tv_su_sv.h
//
//    Contents  : Implementation of tracking_tu_tv_su_sv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking_tu_tv_su_sv.h"

#include <system/memory/memory.h>

#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tracking_tu_tv_su_sv_new(Rox_Tracking_tu_tv_su_sv *tracking, Rox_Tracking_Params params, Rox_Image model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_tu_tv_su_sv ret = NULL;
   Rox_Sint patch_cols, patch_rows;
   Rox_Sint swidth, sheight;
   Rox_PatchPlane lastlevel;
   
   if (!tracking || !params || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *tracking = 0;

   ret = (Rox_Tracking_tu_tv_su_sv)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set internal pointers to NULL
   ret->pyramid = NULL;
   ret->tracker = NULL;
   ret->predicter = NULL;

   // Allocate parent structure 
   error = rox_tracking_alloc(&ret->parent, params, model); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate specific objects
   error = rox_image_get_size(&patch_rows, &patch_cols, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_patchplane_pyramid_new(&ret->pyramid, patch_rows, patch_cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Set model
   error = rox_patchplane_pyramid_apply(ret->pyramid, ret->parent.normalized_ref, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_tracking_patch_tu_tv_su_sv_new(&ret->tracker); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check init_pyr and stop_pyr parameters
   if (ret->parent.init_pyr > (Rox_Sint)(ret->pyramid->count - 1)) ret->parent.init_pyr = (Rox_Sint) (ret->pyramid->count - 1);
   if (ret->parent.init_pyr < 0) ret->parent.init_pyr = 0;

   if (ret->parent.stop_pyr > (Rox_Sint) (ret->pyramid->count - 1)) ret->parent.stop_pyr = (Rox_Sint) (ret->pyramid->count - 1);
   if (ret->parent.stop_pyr < 0) ret->parent.stop_pyr = 0;

   if (ret->parent.stop_pyr > ret->parent.init_pyr) ret->parent.init_pyr = ret->parent.stop_pyr;

   lastlevel = ret->pyramid->levels[ret->parent.init_pyr];

   error = rox_array2d_float_get_size(&sheight, &swidth, lastlevel->reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_search_new(&ret->predicter, sheight, swidth, params->prediction_radius); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_search_set_model(ret->predicter, lastlevel->reference, lastlevel->reference_mask); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // set function pointers
   ret->parent._fptr_make = (Rox_ErrorCode (*)(Rox_Tracking, Rox_Image)) rox_tracking_tu_tv_su_sv_make;
   ret->parent._fptr_del = (Rox_ErrorCode (*)(Rox_Tracking *)) rox_tracking_tu_tv_su_sv_del;
   ret->parent._fptr_set_mask = (Rox_ErrorCode (*) (Rox_Tracking, Rox_Imask))rox_tracking_tu_tv_su_sv_set_mask;

   *tracking = ret;

function_terminate:
   if (error) rox_tracking_tu_tv_su_sv_del(&ret);

   return error;
}

Rox_ErrorCode rox_tracking_tu_tv_su_sv_del(Rox_Tracking_tu_tv_su_sv *tracking)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_tu_tv_su_sv todel;

   if (!tracking) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *tracking;
   *tracking = NULL;

   if (!todel) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_tracking_free(&todel->parent);

   rox_patchplane_pyramid_del(&todel->pyramid);
   rox_plane_search_del(&todel->predicter);
   rox_tracking_patch_tu_tv_su_sv_del(&todel->tracker);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_tu_tv_su_sv_make(Rox_Tracking_tu_tv_su_sv tracking, Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint level = 0;
   Rox_Double prev_score = 0.0;
   Rox_Float alpha = 0.0f, beta = 0.0f;
   Rox_Double tu = 0.0, tv= 0.0, su = 1.0, sv = 1.0;

   if (!tracking || !image)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (tracking->parent.normalized_cur == 0)
   {
      error = rox_array2d_float_new(&tracking->parent.normalized_cur, rows, cols); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Sint cols_parent = 0, rows_parent = 0;
   error = rox_array2d_float_get_size(&rows_parent, &cols_parent, tracking->parent.normalized_cur);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != rows_parent || cols != cols_parent)
   {
      rox_array2d_float_del(&tracking->parent.normalized_cur);
      error = rox_array2d_float_new(&tracking->parent.normalized_cur, rows, cols); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Convert the input image 
   error = rox_array2d_float_from_uchar_normalize(tracking->parent.normalized_cur, image); 
   ROX_ERROR_CHECK_TERMINATE ( error );


   // reset score
   tracking->parent.score = 0.0;

   //Hierarchical tracking using the last warped template
   for (level = tracking->parent.init_pyr; level >= tracking->parent.stop_pyr; level--)
   {
      Rox_PatchPlane patch = NULL;
      patch = tracking->pyramid->levels[level];

      error = rox_transformtools_matrix33_right_pyramidzoom(tracking->parent.zoom_homography, tracking->parent.homography, level); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_tracking_patch_tu_tv_su_sv_from_SL3(&tu, &tv, &su, &sv, tracking->parent.zoom_homography); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_tracking_patch_tu_tv_su_sv_set_state(tracking->tracker, tu, tv, su, sv); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy illumination parameters
      alpha = patch->alpha;
      beta = patch->beta;

      if (level == tracking->parent.init_pyr)
      {
         error = rox_plane_search_make(tracking->predicter, tracking->parent.normalized_cur, tracking->parent.zoom_homography); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_plane_search_update_homography(tracking->parent.zoom_homography, tracking->predicter); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_tracking_patch_tu_tv_su_sv_from_SL3(&tu, &tv, &su, &sv, tracking->parent.zoom_homography); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_tracking_patch_tu_tv_su_sv_set_state(tracking->tracker, tu, tv, su, sv); 
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Compute ZNCC score 
      error = rox_patchplane_prepare_sl3(patch, tracking->parent.zoom_homography, tracking->parent.normalized_cur); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_finish(patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_compute_score(&prev_score, patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_tracking_patch_tu_tv_su_sv_make(tracking->tracker, patch, tracking->parent.normalized_cur, tracking->parent.miter);  
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute ZNCC score 
      error = rox_patchplane_compute_score(&tracking->parent.score, patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      // Check result 
      if(tracking->parent.score > prev_score)
      {
         //error = rox_tracking_patch_tu_tv_su_sv_get_homography(tracking->parent.zoom_homography, tracking->tracker); ROX_ERROR_CHECK_TERMINATE ( error );
         error = rox_tracking_patch_tu_tv_su_sv_get_state(&tu, &tv, &su, &sv, tracking->tracker); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_tracking_patch_tu_tv_su_sv_to_SL3(tracking->parent.zoom_homography, tu, tv, su, sv); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_transformtools_matrix33_right_pyramidzoominv(tracking->parent.homography, tracking->parent.zoom_homography, level); 
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         tracking->parent.score = prev_score;

         // Set the illumination parameters
         patch->alpha = alpha;
         patch->beta = beta;
      }
   }

   if (tracking->parent.score < tracking->parent.min_score)
   {
      //Reset light of each plane level
      for (level = tracking->parent.init_pyr; level >= tracking->parent.stop_pyr; level--)
      {
         Rox_PatchPlane patch;
         patch = tracking->pyramid->levels[level];
         rox_patchplane_reset_luminance(patch);
      }

      error = ROX_ERROR_PROCESS_FAILED;
      ROX_ERROR_CHECK_TERMINATE ( error );
  }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_tu_tv_su_sv_set_mask(Rox_Tracking_tu_tv_su_sv tracking, const Rox_Imask mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!mask || !tracking) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Update the reference mask 
   error = rox_patchplane_pyramid_apply(tracking->pyramid, tracking->parent.normalized_ref, mask); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_PatchPlane lastlevel = tracking->pyramid->levels[tracking->pyramid->count - 1];
   
   error = rox_plane_search_set_model(tracking->predicter, lastlevel->reference, lastlevel->reference_mask); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
