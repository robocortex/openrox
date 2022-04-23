//==============================================================================
//
//    OPENROX   : File odometry_multiplane.c
//
//    Contents  : Implementation of odometry_multiplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_multiplane.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <core/odometry/multiplane/odometry_planes.h>
#include <inout/system/errors_print.h>
#include <user/sensor/camera/camera_struct.h>


Rox_ErrorCode rox_odometry_multi_plane_new ( 
   Rox_Odometry_Multi_Plane * odometry_multi_plane, 
   const Rox_Odometry_Multi_Plane_Params params,
   const Rox_Model_Multi_Plane model 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Multi_Plane ret = NULL;

   if ( !odometry_multi_plane || !model )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   *odometry_multi_plane = NULL;

   error = rox_odometry_planes_new ( &ret, model );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Set default paramaters
   ret->max_iterations = 10;
   ret->score_threshold = 0.89;

   *odometry_multi_plane = ret;

function_terminate:
   if (error) rox_odometry_multi_plane_del( &ret );
   return error;
}


Rox_ErrorCode rox_odometry_multi_plane_del ( 
   Rox_Odometry_Multi_Plane * odometry_multi_plane 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Multi_Plane todel = NULL;

   if ( !odometry_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   todel = *odometry_multi_plane;
   *odometry_multi_plane = NULL;

   if ( !todel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   rox_odometry_planes_del( &todel );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_multi_plane_make (
  Rox_Odometry_Multi_Plane odometry_multi_plane,
  const Rox_Model_Multi_Plane    model,
  const Rox_Camera               camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float nomalized_image = NULL;

   // Test Inputs
   if ( !model || !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Test Outputs
   if ( !odometry_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_odometry_planes_set_camera_calibration ( odometry_multi_plane, camera->calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Normalize input image so that intensity is between 0 and 1
   error = rox_array2d_float_new_from_uchar_normalize ( &nomalized_image, camera->image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // max_iterations should be a parameter
   error = rox_odometry_planes_make ( odometry_multi_plane, model, nomalized_image );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del( &nomalized_image );
   return error;
}


Rox_ErrorCode rox_odometry_multi_plane_set_pose ( 
   Rox_Odometry_Multi_Plane odometry, 
   const Rox_MatSE3 pose 
)
{
   return rox_odometry_planes_set_pose( odometry, pose );
}


Rox_ErrorCode rox_odometry_multi_plane_get_pose ( 
   Rox_MatSE3 pose, 
   const Rox_Odometry_Multi_Plane odometry_multi_plane 
)
{
   return rox_odometry_planes_get_pose( pose, odometry_multi_plane );
}

Rox_ErrorCode rox_odometry_multi_plane_get_score (
   Rox_Double * score, 
   const Rox_Odometry_Multi_Plane odometry_multi_plane 
)
{
   return rox_odometry_planes_get_score( score, odometry_multi_plane );
}

Rox_ErrorCode rox_odometry_multi_plane_get_result ( 
   Rox_Sint * is_tracked, 
   Rox_Double * score, 
   Rox_MatSE3 pose, 
   const Rox_Odometry_Multi_Plane odometry_multi_plane 
)
{
   return rox_odometry_planes_get_result ( is_tracked, score, pose, odometry_multi_plane );
}

Rox_ErrorCode rox_odometry_multi_plane_set_score_threshold (
   Rox_Odometry_Multi_Plane odometry_multi_plane, 
   const Rox_Double score_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry_multi_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   odometry_multi_plane->score_threshold = score_threshold;

function_terminate:
   return error;
}
