//==============================================================================
//
//    OPENROX   : File odometry_singleplane.c
//
//    Contents  : Implementation of odometry_singleplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <float.h>

#include "odometry_singleplane.h"
#include "odometry_singleplane_struct.h"
#include "odometry_singleplane_params.h"
#include "odometry_singleplane_params_struct.h"
#include "odometry_singleplane_light_affine.h"
#include "odometry_singleplane_light_affine_struct.h"
#include "odometry_singleplane_light_robust.h"
#include "odometry_singleplane_light_robust_struct.h"

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/point/point3d_tools.h>

//#include <core/model/model_single_plane_struct.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_single_plane_new (
   Rox_Odometry_Single_Plane * odometry, 
   const Rox_Odometry_Single_Plane_Params params, 
   const Rox_Model_Single_Plane model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane ret = NULL;

   if ( !odometry )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !params || !model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry = NULL;

   switch (params->usecase)
   {
      case Rox_Odometry_Single_Plane_UseCase_Affine_Light:
      {
         Rox_Odometry_Single_Plane_Light_Affine ret_affine = NULL;

         error = rox_odometry_single_plane_light_affine_new ( &ret_affine, params, model );
         ROX_ERROR_CHECK_TERMINATE(error)

         ret = (Rox_Odometry_Single_Plane) ret_affine;
         break;
      }
      case Rox_Odometry_Single_Plane_UseCase_Robust_Light:
      {
         Rox_Odometry_Single_Plane_Light_Robust ret_robust = NULL;

         error = rox_odometry_single_plane_light_robust_new ( &ret_robust, params, model );
         ROX_ERROR_CHECK_TERMINATE(error)

         ret = (Rox_Odometry_Single_Plane) ret_robust;
         break;
      }
      default:
      {  error = ROX_ERROR_INVALID_VALUE; 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }
   }

   // Copy the vertices of the model
   error = rox_vector_point3d_double_copy ( ret->model_corners, model->vertices_ref, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *odometry = ret;

function_terminate:
   // Delete memory only if an error occurs
   if (error) rox_odometry_single_plane_del(&ret);

   return error;
}


Rox_ErrorCode rox_odometry_single_plane_del
(
   Rox_Odometry_Single_Plane * odometry
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Single_Plane todel = NULL;

   if (!odometry)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *odometry;
   *odometry = NULL;

   if(!todel)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   if (!todel->_fptr_del)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   // call del function
   error = todel->_fptr_del(&todel);

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_alloc (
   Rox_Odometry_Single_Plane odometry, 
   Rox_Odometry_Single_Plane_Params params, 
   Rox_Model_Single_Plane model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry || !params || !model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Set to NULL pointers
   odometry->normalized_ref       = NULL;
   odometry->normalized_cur       = NULL;
   odometry->calibration_template = NULL;
   odometry->zoom_calibration     = NULL;
   odometry->pose                 = NULL;
   odometry->posebuffer           = NULL;
   odometry->homography           = NULL;
   odometry->_fptr_del            = NULL;
   odometry->_fptr_make           = NULL;
   odometry->_fptr_set_mask       = NULL;

   error = rox_matse3_new ( &odometry->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &odometry->posebuffer );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &odometry->calibration_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &odometry->zoom_calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &odometry->homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Convert model image from uchar to float
   error = rox_array2d_float_new_from_uchar_normalize(&odometry->normalized_ref, model->image_template);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Obvious calibration from dimensions
   Rox_Sint patch_cols = 0, patch_rows = 0;
   error = rox_array2d_uchar_get_size ( &patch_rows, &patch_cols, model->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO: to be replaced by a more generic function computing the "calibration matrix" for template 
   Rox_Double sizx = model->vertices_ref[1].X - model->vertices_ref[0].X;
   Rox_Double sizy = model->vertices_ref[2].Y - model->vertices_ref[1].Y;
   error = rox_transformtools_build_calibration_matrix_for_template ( odometry->calibration_template, patch_cols, patch_rows, sizx, sizy);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set Default parameters
   odometry->score             = 0.0;
   odometry->miter             = 10;
   odometry->min_score         = 0.89;
   odometry->prediction_radius = params->prediction_radius;
   odometry->init_pyr          = params->init_pyr;
   odometry->stop_pyr          = params->stop_pyr;

function_terminate:
   if (error) rox_odometry_single_plane_free(odometry);
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_free (
   Rox_Odometry_Single_Plane odometry
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&odometry->pose);
   rox_matse3_del(&odometry->posebuffer);

   rox_matut3_del(&odometry->calibration_template);
   rox_matut3_del(&odometry->zoom_calibration);
   rox_matsl3_del(&odometry->homography);

   rox_array2d_float_del(&odometry->normalized_ref);
   rox_array2d_float_del(&odometry->normalized_cur);

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_make (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry || !camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Call make function
   error = odometry->_fptr_make(odometry, camera);

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_get_score (
   Rox_Double *score, 
   const Rox_Odometry_Single_Plane odometry
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!score || !odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = odometry->score;

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_set_pose (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry || !pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(odometry->pose, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_set_score_thresh (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Double score_thresh
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   odometry->min_score = score_thresh;

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_set_miter (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Sint miter
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (miter == 0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   odometry->miter = miter;

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_get_pose (
   Rox_MatSE3                 pose, 
   Rox_Odometry_Single_Plane  odometry
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose || !odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy(pose, odometry->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_get_model_visibility (
   Rox_Sint * visibility,
   const Rox_Odometry_Single_Plane odometry,
   const Rox_Camera camera 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check Outputs
   if ( !visibility )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check Inputs
   if ( !odometry || !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get image size
   Rox_Sint image_cols = 0, image_rows = 0;
   error = rox_camera_get_size ( &image_rows, &image_cols, camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get intrinsic parameters
   Rox_MatUT3 Kc = NULL;
   error = rox_camera_get_intrinsic_parameters_pointer ( &Kc, camera ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pose
   Rox_MatSE3 cTo = odometry->pose;

   // Get model points
   Rox_Point3D_Double mo = odometry->model_corners;

   error = rox_vector_point3d_double_check_visibility ( visibility, image_rows, image_cols, Kc, cTo, mo, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_single_plane_set_mask (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!mask || !odometry)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Call set_mask function
   error = odometry->_fptr_set_mask(odometry, mask);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}
