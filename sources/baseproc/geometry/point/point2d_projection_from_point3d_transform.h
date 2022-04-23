//==============================================================================
//
//    OPENROX   : File point2d_projection_from_point3d_transform.h
//
//    Contents  : API of point2d_projection_from_point3d_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS_TRANSFORM_PROJECT__
#define __OPENROX_POINTS_TRANSFORM_PROJECT__

#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point3d.h>

//! \addtogroup Point2D
//! @{

//! Given a list of points, transform and project them as viewed for a given pose
//! \param [out]  res      Result pixel coordinates
//! \param [in ]  pose     Camera pose matrix
//! \param [in ]  calib    Matrix of the camera
//! \param [in ]  input    A points list to transform
//! \param [in ]  count    The list size
//! \return An error code
//! \ŧodo   To be tested, rename to rox_point2d_float_transformproject_point3d_float
ROX_API Rox_ErrorCode rox_point3d_float_transform_project (
   Rox_Point2D_Float res, 
   Rox_MatSE3 pose, 
   Rox_MatUT3 calib, 
   Rox_Point3D_Float input, 
   Rox_Uint count);

//! Given a list of points, transform and project them as viewed for a given pose
//! \param  [out]  res      Result pixel coordinates
//! \param  [in ]  pose     Camera pose matrix
//! \param  [in ]  calib    Matrix of the camera
//! \param  [in ]  input    A points list to transform
//! \param  [in ]  count    The list size
//! \return An error code
//! \ŧodo   To be tested, rename to rox_point2d_double_transformproject_point3d_double
ROX_API Rox_ErrorCode rox_point3d_double_transform_project(
   Rox_Point2D_Double res, 
   Rox_MatSE3 pose, 
   Rox_MatUT3 calib, 
   Rox_Point3D_Double input, 
   Rox_Uint count
);

//! Given a list of points, transform and project them as viewed for a given pose in meters
//! \param  [out]  res      Result pixel coordinates
//! \param  [in ]  pose     Camera pose matrix
//! \param  [in ]  input    A points list to transform
//! \param  [in ]  count    The list size
//! \return An error code
//! \ŧodo   To be tested, rename to rox_point2d_float_transformproject_meters_point3d_float
ROX_API Rox_ErrorCode rox_point3d_float_transform_project_meters (
   Rox_Point2D_Float res, 
   Rox_MatSE3 pose, 
   Rox_Point3D_Float input, 
   Rox_Uint count
);

//! Given a list of points, transform and project them as viewed for a given pose in meters
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  pose           Camera pose matrix
//! \param  [in ]  input          A points list to transform
//! \param  [in ]  count          The list size
//! \return An error code
//! \ŧodo   To be tested, rename to rox_point2d_double_transformproject_meters_point3d_double
ROX_API Rox_ErrorCode rox_point3d_double_transform_project_meters (
   Rox_Point2D_Double res, 
   Rox_MatSE3 pose, 
   Rox_Point3D_Double input, 
   Rox_Uint count
);

//! Given a list of 3D points on a plane with Z = 0, transform and project them as viewed for a given model-to-image homography cHo
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  homography     Homography matrix
//! \param  [in ]  input          A points list to transform
//! \param  [in ]  count          The list size
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_point3d_float_transform_project_homography (
   Rox_Point2D_Float res, 
   Rox_MatSL3 homography, 
   Rox_Point3D_Float input, 
   Rox_Uint count
);

//! Given a list of 3D points on a plane with Z = 0, transform and project them as viewed for a given model-to-image homography cHo
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  homography     Camera pose matrix
//! \param  [in ]  input          A points list to transform
//! \param  [in ]  count          The list size
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_point3d_double_transform_project_homography (
   Rox_Point2D_Double res, 
   Rox_MatSL3 homography, 
   Rox_Point3D_Double input, 
   Rox_Uint count
);

//! @}

#endif
