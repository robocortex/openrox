//==============================================================================
//
//    OPENROX   : File odometry_singleplane_sparse.c
//
//    Contents  : Implementation of odometry_singleplane_sparse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_singleplane_sparse.h"

#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_singleplane_sparse (
   Rox_MatSE3 pose, 
   const Rox_MatUT3 K, 
   const Rox_Point2D_Double p, 
   const Rox_Double size_x, 
   const Rox_Double size_y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE; 
   Rox_MatSL3 homography = NULL;
   Rox_Point2D_Double_Struct n[4];

   // Define the 2D rectangle coordinates
   error = rox_rectangle2d_create_centered_plane_xright_ydown ( n, size_x, size_y );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &homography );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // compute homography such that p = homography * n
   error = rox_matsl3_from_4_points_double ( homography, n, p );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_pose_intermodel(pose, homography, K);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   rox_matsl3_del(&homography);
   return error;
}