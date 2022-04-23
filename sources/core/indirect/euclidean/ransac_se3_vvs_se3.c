//==============================================================================
//
//    OPENROX   : File ransac_se3_vvs_se3.c
//
//    Contents  : Implementation of ransac_se3_vvs_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ransac_se3_vvs_se3.h"

#include <core/indirect/euclidean/ransacse3.h>
#include <core/indirect/euclidean/vvspointsse3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ransac_se3_vvs_se3_float (
  Rox_MatSE3               pose,
  Rox_DynVec_Point2D_Float inliers2D,
  Rox_DynVec_Point3D_Float inliers3D,
  Rox_DynVec_Point2D_Float input2D,
  Rox_DynVec_Point3D_Float input3D,
  Rox_MatSL3               calib,
  Rox_Double                 maxdist_prefilter 
)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   error = rox_ransac_p3p( pose, inliers2D, inliers3D, input2D, input3D, calib );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_points_float_refine_pose_vvs( pose, calib, inliers2D, inliers3D, maxdist_prefilter );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:

   return error;
}


Rox_ErrorCode rox_ransac_se3_vvs_se3_double (
  Rox_MatSE3                pose,
  Rox_DynVec_Point2D_Double inliers2D,
  Rox_DynVec_Point3D_Double inliers3D,
  Rox_DynVec_Point2D_Double input2D,
  Rox_DynVec_Point3D_Double input3D,
  Rox_MatSL3                calib,
  Rox_Double                  maxdist_prefilter 
)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   error = rox_ransac_p3p_double( pose, inliers2D, inliers3D, input2D, input3D, calib );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_points_double_refine_pose_vvs( pose, calib, inliers2D, inliers3D, maxdist_prefilter );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:

   return error;
}
