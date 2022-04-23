//==============================================================================
//
//    OPENROX   : File odometry_singleplane_light_affine.c
//
//    Contents  : Implementation of odometry_singleplane_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_singleplane_light_affine.h"
#include "odometry_singleplane_light_affine_struct.h"
#include "odometry_singleplane_params_struct.h"

#include <stdio.h>

#include <system/memory/memory.h>
#include <system/time/timer.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>

//#include <core/model/model_single_plane_struct.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>


Rox_ErrorCode rox_odometry_single_plane_light_affine_new (
   Rox_Odometry_Single_Plane_Light_Affine * odometry,
   const Rox_Odometry_Single_Plane_Params params,
   const Rox_Model_Single_Plane model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane_Light_Affine ret = NULL;
   Rox_Sint                               patch_cols = 0, patch_rows = 0;
   Rox_Sint                               swidth = 0, sheight = 0;
   Rox_PatchPlane                         lastlevel = NULL;

   if (!odometry || !params || !model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry = NULL;

   ret = (Rox_Odometry_Single_Plane_Light_Affine) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set internal pointers to NULL
   ret->pyramid = NULL;
   ret->tracker = NULL;
   ret->predicter = NULL;

   // Allocate parent structure
   error = rox_odometry_single_plane_alloc ( &ret->parent, params, model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate specific objects
   error = rox_array2d_uchar_get_size ( &patch_rows, &patch_cols, model->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_patchplane_pyramid_new ( &ret->pyramid, patch_rows, patch_cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set model
   error = rox_patchplane_pyramid_apply ( ret->pyramid, ret->parent.normalized_ref, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_odometry_plane_new ( &ret->tracker );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check init_pyr and stop_pyr parameters
   if (ret->parent.init_pyr > (Rox_Sint) (ret->pyramid->count - 1)) ret->parent.init_pyr = (Rox_Sint) (ret->pyramid->count - 1);
   if (ret->parent.init_pyr < 0) ret->parent.init_pyr = 0;

   if (ret->parent.stop_pyr > (Rox_Sint) (ret->pyramid->count - 1)) ret->parent.stop_pyr = (Rox_Sint) (ret->pyramid->count - 1);

   if (ret->parent.stop_pyr < 0) ret->parent.stop_pyr = 0;

   if (ret->parent.stop_pyr > ret->parent.init_pyr) ret->parent.init_pyr = ret->parent.stop_pyr;

   lastlevel = ret->pyramid->levels[ret->parent.init_pyr];
   error = rox_array2d_float_get_size ( &sheight, &swidth, lastlevel->reference );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_search_new ( &ret->predicter, sheight, swidth, params->prediction_radius);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_search_set_model ( ret->predicter, lastlevel->reference, lastlevel->reference_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set function pointers
   ret->parent._fptr_make = (Rox_ErrorCode (*)(Rox_Odometry_Single_Plane, Rox_Camera)) rox_odometry_single_plane_light_affine_make;
   ret->parent._fptr_del = (Rox_ErrorCode (*)(Rox_Odometry_Single_Plane *)) rox_odometry_single_plane_light_affine_del;
   ret->parent._fptr_set_mask = (Rox_ErrorCode (*) (Rox_Odometry_Single_Plane, Rox_Imask)) rox_odometry_single_plane_light_affine_set_mask;

   *odometry = ret;

function_terminate:
   if (error) rox_odometry_single_plane_light_affine_del(&ret);

   return error;
}


Rox_ErrorCode rox_odometry_single_plane_light_affine_del (
   Rox_Odometry_Single_Plane_Light_Affine * odometry
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane_Light_Affine todel = NULL;

   if (!odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry;
   *odometry = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_odometry_single_plane_free(&todel->parent);

   rox_patchplane_pyramid_del(&todel->pyramid);
   rox_plane_search_del(&todel->predicter);
   rox_odometry_plane_del(&todel->tracker);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

// #define POSE_SHIFT_TZ1 0 // NEW STYLE
#define POSE_SHIFT_TZ1 1 // OLD SYTLE OK

Rox_ErrorCode rox_odometry_single_plane_light_affine_make (
   Rox_Odometry_Single_Plane_Light_Affine odometry,
   const Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint level = 0;
   Rox_Double prev_score = 0.0;
   Rox_Float alpha = 1.0f, beta = 0.0f;

   if ( !odometry )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test if the normalized image has already been created
   if(odometry->parent.normalized_cur == NULL)
   {
      Rox_Sint cols = 0, rows = 0;
      error = rox_array2d_uchar_get_size ( &rows, &cols, camera->image );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_new ( &odometry->parent.normalized_cur, rows, cols );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Convert the input image to float and normalize values between 0 and 1
   error = rox_array2d_float_from_uchar_normalize ( odometry->parent.normalized_cur, camera->image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reset score
   odometry->parent.score = 0.0;

   if(POSE_SHIFT_TZ1 == 1)
   {
      error = rox_transformtools_updateZref ( odometry->parent.posebuffer, odometry->parent.pose, -1);
      ROX_ERROR_CHECK_TERMINATE ( error);
   }
   else
   {
      error = rox_matse3_copy ( odometry->parent.posebuffer, odometry->parent.pose );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Hierarchical tracking over a pyramid of images
   for (level = odometry->parent.init_pyr; level >= odometry->parent.stop_pyr; level--)
   {
      Rox_PatchPlane patch = odometry->pyramid->levels[level];

      // Set the zoom calibration matrix for the given level
      error = rox_transformtools_matrix33_left_pyramidzoom ( odometry->parent.zoom_calibration, odometry->parent.calibration_template, level);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_odometry_plane_set_pose ( odometry->tracker, odometry->parent.posebuffer, camera->calib_camera, odometry->parent.zoom_calibration);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy illumination parameters
      alpha = patch->alpha;
      beta = patch->beta;

      // If we are at the initial level in the pyramid make a prediction
      if (level == odometry->parent.init_pyr)
      {
         // Build the image to image homography c_G_t = c_G_o * inv(o_G_t)
         if ( POSE_SHIFT_TZ1 == 1 )
         {
            error = rox_transformtools_build_homography(odometry->parent.homography, odometry->parent.posebuffer, camera->calib_camera, odometry->parent.zoom_calibration, 0, 0, -1, 1);
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
         else
         {
            // New code to replace rox_transformtools_build_homography function

            Rox_MatUT3 Kc    = camera->calib_camera;
            Rox_MatSE3 c_T_o = odometry->parent.pose;
            Rox_MatSL3 t_G_o = odometry->parent.zoom_calibration; // -> Kr
            Rox_MatSL3 c_G_t = odometry->parent.homography;

            Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

            error = rox_matsl3_new ( &c_G_o ) ;
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
            ROX_ERROR_CHECK_TERMINATE ( error );

            rox_matsl3_del ( &c_G_o );
         }

         // Search plane to predict pose
         error = rox_plane_search_make ( odometry->predicter, odometry->parent.normalized_cur, odometry->parent.homography);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Display search results
         Rox_Double score = 0.0, tu = 0.0, tv = 0.0;
         {
            error = rox_plane_search_get_results ( &score, &tu, &tv, odometry->predicter);
            ROX_ERROR_CHECK_TERMINATE ( error );
         }

         error = rox_plane_search_update_pose_translation ( odometry->parent.posebuffer, odometry->parent.zoom_calibration, odometry->predicter);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_odometry_plane_set_pose ( odometry->tracker, odometry->parent.posebuffer, camera->calib_camera, odometry->parent.zoom_calibration);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Compute normalized ZNCC score

      // Compute c_G_t homography with the updated pose c_T_o

      if(POSE_SHIFT_TZ1 == 1)
      {
         error = rox_transformtools_build_homography ( odometry->tracker->homography, odometry->tracker->pose, odometry->tracker->calibration_camera, odometry->tracker->calibration_template, 0, 0, -1, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      else
      {
         // New code to replace rox_transformtools_build_homography function
         Rox_MatUT3 Kc    = odometry->tracker->calibration_camera;
         Rox_MatSE3 c_T_o = odometry->tracker->pose;
         Rox_MatSL3 t_G_o = odometry->tracker->calibration_template;
         Rox_MatSL3 c_G_t = odometry->tracker->homography;
         Rox_MatSL3 c_G_o = NULL;

         error = rox_matsl3_new ( &c_G_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_matsl3_del ( &c_G_o );
      }

      error = rox_patchplane_prepare_sl3 ( patch, odometry->tracker->homography, odometry->parent.normalized_cur );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_prepare_finish ( patch );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_compute_score ( &prev_score, patch );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Make tracking
      error = rox_odometry_plane_make ( odometry->tracker, patch, odometry->parent.normalized_cur, odometry->parent.miter );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute normlized ZNCC score between 0 and 1
      error = rox_patchplane_compute_score ( &odometry->parent.score, patch );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Check result
      if ( odometry->parent.score > prev_score )
      {
         error = rox_odometry_plane_get_pose ( odometry->parent.posebuffer, odometry->tracker );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         odometry->parent.score = prev_score;

         // Set the illumination parameters
         patch->alpha = alpha;
         patch->beta = beta;
      }

      if (level>0)
      {
         Rox_PatchPlane patch_next = odometry->pyramid->levels[level-1];
         patch_next->alpha = patch->alpha;
         patch_next->beta = patch->beta;
      }
   }

   if(POSE_SHIFT_TZ1 == 1)
   {
      error = rox_transformtools_updateZref ( odometry->parent.pose, odometry->parent.posebuffer, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_matse3_copy ( odometry->parent.pose, odometry->parent.posebuffer );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (odometry->parent.score < odometry->parent.min_score)
   {
      // Reset light of each plane level
      for (level = odometry->parent.init_pyr; level >= odometry->parent.stop_pyr; level--)
      {
         Rox_PatchPlane patch;
         patch = odometry->pyramid->levels[level];
         rox_patchplane_reset_luminance ( patch );
      }

      error = ROX_ERROR_PROCESS_FAILED;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_light_affine_set_mask (
   Rox_Odometry_Single_Plane_Light_Affine odometry,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !odometry )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !mask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Update the reference mask
   error = rox_patchplane_pyramid_apply ( odometry->pyramid, odometry->parent.normalized_ref, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_PatchPlane lastlevel = odometry->pyramid->levels[odometry->pyramid->count - 1];

   error = rox_plane_search_set_model ( odometry->predicter, lastlevel->reference, lastlevel->reference_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
