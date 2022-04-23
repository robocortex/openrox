//==============================================================================
//
//    OPENROX   : File point3d_inverse_projection_from_point2d.h
//
//    Contents  : API of inverse_projection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS3D_INVERSE_PROJECTION_FROM_POINTS2D__
#define __OPENROX_POINTS3D_INVERSE_PROJECTION_FROM_POINTS2D__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/plane/plane_struct.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>

//! compute the inverse projection of coplanar points, given a list of 2d points, 
//! the camera calibration, the pose and the 3D plane
//! \param  [out]  points3d       The 3D points list (all points are suppose to be coplanar)
//! \param  [in ]  plane          The 3D plane on which the 3D points are supposed to be
//! \param  [in ]  calib          The camera calibration matrix
//! \param  [in ]  pose           The pose
//! \param  [in ]  input          The 2D points list 
//! \param  [in ]  nbpts          The 2D points list size
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point3d_double_coplanar_inverse_projection_from_point2d_double (
   Rox_Point3D_Double mo,  
   const Rox_Plane3D_Double plane3d_o,
   const Rox_MatUT3 Kc, 
   const Rox_MatSE3 cTo,
   const Rox_Point2D_Double pc, 
   const Rox_Sint nbpts
);

#endif
