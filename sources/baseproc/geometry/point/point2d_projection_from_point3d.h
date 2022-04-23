//==============================================================================
//
//    OPENROX   : File point2d_projection_from_point3d.h
//
//    Contents  : API of pointsproject module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS2D_PROJECTION_FROM_POINTS3D__
#define __OPENROX_POINTS2D_PROJECTION_FROM_POINTS3D__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>

//! Given a point, compute the projection of the points for a given pose
//! \param  [out]  output2d       Result pixels coordinates
//! \param  [in ]  input3d        A point list to project
//! \param  [in ]  K              Camera calibration matrix
//! \param  [in ]  nbpts          The number of points (must be the same for input3d and output2d)
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_double_project ( 
   Rox_Point2D_Double output2d, 
   Rox_Point3D_Double input3d, 
   const Rox_MatUT3 K, 
   const Rox_Sint nbpts
);

//! Given a point, compute the projection of the points for a given pose
//! \param  [out]  output2d       Result pixels coordinates
//! \param  [in ]  input          A point list to project
//! \param  [in ]  calib          Camera calibration matrix
//! \param  [in ]  nbpts          The point list size
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point2d_float_project (
   Rox_Point2D_Float output2d, 
   Rox_Point3D_Float input, 
   const Rox_MatUT3 calib, 
   const Rox_Sint nbpts
);

ROX_API Rox_ErrorCode rox_point2d_float_transform_project (
   Rox_Point2D_Float res, Rox_Float * depth, 
   const Rox_MatUT3 calib, 
   const Rox_MatSE3 pose, 
   const Rox_Point3D_Float input, 
   const Rox_Sint nbpts
);

//! Given a point, compute the projection in meters of the points for a given pose
//! \param  [out]  output2d       Result meters coordinates
//! \param  [in ]  input          A point list to project
//! \param  [in ]  nbpts          The point list size
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point2d_float_project_meters (
   Rox_Point2D_Float output2d, 
   const Rox_Point3D_Float input, 
   const Rox_Sint nbpts
);

ROX_API Rox_ErrorCode rox_point2d_float_transform_project_meters (
   Rox_Point2D_Float q, Rox_Float * depth, 
   const Rox_MatSE3 pose, 
   const Rox_Point3D_Float input, 
   const Rox_Sint nbpts
);

//! Given a point, compute the projection in pixels, plus depth of the points for a given pose
//! \param  [out]  res            Result pixel coordinates
//! \param  [out]  depth          Result depth in meters per point
//! \param  [in ]  pose           The pose to use for transformation
//! \param  [in ]  calib          The calibration matrix used for projection
//! \param  [in ]  input          A point list to project
//! \param  [in ]  nbpts          The point list size
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_point2d_double_transform_project (
   Rox_Point2D_Double res, 
   Rox_Double * depth, 
   const Rox_MatUT3 calib, 
   const Rox_MatSE3 pose, 
   const Rox_Point3D_Double input, 
   const Rox_Sint nbpts
);

//!
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  model          A point list to project
//! \param  [in ]  nbp            The number of points in the list
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_point2d_double_intermodel_projection (
   Rox_Point2D_Double res, 
   const Rox_MatSL3 homography, 
   const Rox_Point3D_Double model, 
   const Rox_Sint nbpts
);

//!
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  model          A point list to project
//! \param  [in ]  nbpts          The point list size
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_point2d_float_intermodel_projection (
   Rox_Point2D_Float res, 
   const Rox_MatSL3 homography, 
   const Rox_Point3D_Double model, 
   const Rox_Sint nbpts
);

#endif
