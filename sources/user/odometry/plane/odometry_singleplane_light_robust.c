//==============================================================================
//
//    OPENROX   : File odometry_singleplane_light_robust.c
//
//    Contents  : Implementation of odometry_singleplane_light_robust module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_singleplane_light_robust.h"
#include "odometry_singleplane_light_robust_struct.h"
#include "odometry_singleplane_params_struct.h"

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>

#include <core/model/model_single_plane_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>

Rox_ErrorCode rox_odometry_single_plane_light_robust_new (
   Rox_Odometry_Single_Plane_Light_Robust * odometry,
   Rox_Odometry_Single_Plane_Params params,
   Rox_Model_Single_Plane model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane_Light_Robust ret = NULL;
   Rox_Sint patch_cols, patch_rows;
   Rox_Sint swidth, sheight;
   Rox_PatchPlane_RobustLight lastlevel;

   if (!odometry || !params || !model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry = NULL;

   ret = (Rox_Odometry_Single_Plane_Light_Robust)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set internal pointers to 0
   ret->pyramid = 0;
   ret->tracker = 0;
   ret->predicter = 0;

   // Allocate parent structure
   error = rox_odometry_single_plane_alloc(&ret->parent, params, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate specific objects
   error = rox_array2d_uchar_get_size(&patch_rows, &patch_cols, model->image_template);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_patchplane_robustlight_pyramid_new(&ret->pyramid, patch_rows, patch_cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set model
   error = rox_patchplane_robustlight_pyramid_apply(ret->pyramid, ret->parent.normalized_ref, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_odometry_plane_robustlight_new(&ret->tracker);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check init_pyr and stop_pyr parameters
   if (ret->parent.init_pyr > (Rox_Sint) (ret->pyramid->count - 1)) ret->parent.init_pyr = (Rox_Sint) (ret->pyramid->count - 1);
   if (ret->parent.init_pyr < 0) ret->parent.init_pyr = 0;

   if (ret->parent.stop_pyr > (Rox_Sint)(ret->pyramid->count - 1)) ret->parent.stop_pyr = (Rox_Sint) (ret->pyramid->count - 1);
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
   ret->parent._fptr_make     = (Rox_ErrorCode (*) (Rox_Odometry_Single_Plane, Rox_Camera)) rox_odometry_single_plane_light_robust_make;
   ret->parent._fptr_del      = (Rox_ErrorCode (*) (Rox_Odometry_Single_Plane *)) rox_odometry_single_plane_light_robust_del;
   ret->parent._fptr_set_mask = (Rox_ErrorCode (*) (Rox_Odometry_Single_Plane, Rox_Imask)) rox_odometry_single_plane_light_robust_set_mask;

   *odometry = ret;

function_terminate:
   if (error) rox_odometry_single_plane_light_robust_del(&ret);

   return error;
}

Rox_ErrorCode rox_odometry_single_plane_light_robust_del(Rox_Odometry_Single_Plane_Light_Robust *odometry)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane_Light_Robust todel = NULL;

   if (!odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry;
   *odometry = 0;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_odometry_single_plane_free(&todel->parent);

   rox_patchplane_robustlight_pyramid_del(&todel->pyramid);
   rox_plane_search_del(&todel->predicter);
   rox_odometry_plane_robustlight_del(&todel->tracker);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_single_plane_light_robust_make (
   Rox_Odometry_Single_Plane_Light_Robust odometry,
   Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double prev_score = 0.0;
   Rox_Double beta = 0.0;
   Rox_DynVec_Double alphas = 0;

   if (!odometry || !camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(odometry->parent.normalized_cur == 0)
   {
      Rox_Sint cols = 0, rows = 0;
      error = rox_array2d_uchar_get_size(&rows, &cols, camera->image);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_new(&odometry->parent.normalized_cur, rows, cols);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Convert the input image
   error = rox_array2d_float_from_uchar_normalize(odometry->parent.normalized_cur, camera->image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reset score
   odometry->parent.score = 0.0;

   error = rox_transformtools_updateZref(odometry->parent.posebuffer, odometry->parent.pose, -1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&alphas, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Hierarchical tracking
   for (Rox_Sint level = odometry->parent.init_pyr; level >= odometry->parent.stop_pyr; level--)
   {
      Rox_PatchPlane_RobustLight patch = odometry->pyramid->levels[level];

      error = rox_transformtools_matrix33_left_pyramidzoom(odometry->parent.zoom_calibration, odometry->parent.calibration_template, level);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_odometry_plane_robustlight_set_pose(odometry->tracker, odometry->parent.posebuffer, camera->calib_camera, odometry->parent.zoom_calibration);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy illumination parameters
      beta = patch->beta;

      error = rox_dynvec_double_clone(alphas, patch->alphas);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (level == odometry->parent.init_pyr)
      {
         error = rox_transformtools_build_homography(odometry->parent.homography, odometry->parent.posebuffer, camera->calib_camera, odometry->parent.zoom_calibration, 0, 0, 1, -1);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_plane_search_make(odometry->predicter, odometry->parent.normalized_cur, odometry->parent.homography);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_plane_search_update_pose_translation(odometry->parent.posebuffer, odometry->parent.zoom_calibration, odometry->predicter);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_odometry_plane_robustlight_set_pose(odometry->tracker, odometry->parent.posebuffer, camera->calib_camera, odometry->parent.zoom_calibration);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Compute ZNCC score
      error = rox_transformtools_build_homography(odometry->tracker->homography, odometry->tracker->pose, odometry->tracker->calibration_camera, odometry->tracker->calibration_template, 0, 0, -1, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_robustlight_prepare_sl3(patch, odometry->tracker->homography, odometry->parent.normalized_cur);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_robustlight_prepare_finish(patch);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_robustlight_compute_score(&prev_score, patch);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Make tracking
      error = rox_odometry_plane_robustlight_make(odometry->tracker, patch, odometry->parent.normalized_cur, odometry->parent.miter);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute ZNCC score
      error = rox_patchplane_robustlight_compute_score(&odometry->parent.score, patch);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Check result
      if (odometry->parent.score > prev_score)
      {
         error = rox_odometry_plane_robustlight_get_pose(odometry->parent.posebuffer, odometry->tracker);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         odometry->parent.score = prev_score;

         // Set the illumination parameters
         error = rox_dynvec_double_clone(patch->alphas, alphas);
         ROX_ERROR_CHECK_TERMINATE ( error );

         patch->beta = (Rox_Float) beta;
      }
   }
   error = rox_transformtools_updateZref(odometry->parent.pose, odometry->parent.posebuffer, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (odometry->parent.score < odometry->parent.min_score)
   {
      // Reset light of each plane level
      for (Rox_Sint level = odometry->parent.init_pyr; level >= odometry->parent.stop_pyr; level--)
      {
         Rox_PatchPlane_RobustLight patch;
         patch = odometry->pyramid->levels[level];
         rox_patchplane_robustlight_reset_luminance(patch);
      }
      error = ROX_ERROR_PROCESS_FAILED;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_dynvec_double_del(&alphas);
   return error;
}

Rox_ErrorCode rox_odometry_single_plane_light_robust_set_mask(Rox_Odometry_Single_Plane_Light_Robust odometry, const Rox_Imask mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!mask || !odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Update the reference mask
   error = rox_patchplane_robustlight_pyramid_apply(odometry->pyramid, odometry->parent.normalized_ref, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_PatchPlane_RobustLight lastlevel = odometry->pyramid->levels[odometry->pyramid->count - 1];
   error = rox_plane_search_set_model(odometry->predicter, lastlevel->reference, lastlevel->reference_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
