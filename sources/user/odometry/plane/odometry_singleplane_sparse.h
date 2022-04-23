//==============================================================================
//
//    OPENROX   : File odometry_singleplane_sparse.h
//
//    Contents  : API of odometry_singleplane_sparse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_SPARSE__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_SPARSE__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

#include <baseproc/geometry/point/point2d.h>

//! Compute the pose from 4 image points and a known 3D rectangle of size_x and size_y
//! \param  [out]  pose           The estimated pose    
//! \param  [in ]  K              The camera perspective intrinsic parameters 
//! \param  [in ]  p              The 4 image points in pixels
//! \param  [in ]  size_x         The size of the rectangle along the x-axis
//! \param  [in ]  size_y         The size of the rectangle along the y-axis
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_singleplane_sparse (
   Rox_MatSE3 pose, 
   const Rox_MatUT3 K, 
   const Rox_Point2D_Double p, 
   const Rox_Double size_x, 
   const Rox_Double size_y
);

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_SPARSE__
