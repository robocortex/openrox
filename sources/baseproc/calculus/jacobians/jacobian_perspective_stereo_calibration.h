//==============================================================================
//
//    OPENROX   : File perspective_stereo_calibration.h
//
//    Contents  : API of perspective_stereo_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PERSPECTIVE_STEREO_CALIBRATION__
#define __OPENROX_PERSPECTIVE_STEREO_CALIBRATION__

#include <generated/array2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup Jacobians
//! \defgroup Calibration_Jacobians Calibration_Jacobians

//! \addtogroup Calibration_Jacobians
//! @{

//! Compute the jacobian for stereo camera calibration (fu, fv, cu, cv, skew) wrt 2d point list
//! \param  [out]  Jk             The Jacobian
//! \param  [in ]  pts            List of 2D points
//! \param  [in ]  nbpts          Length of the 2D point list
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_perspective_stereo_calibration (
   Rox_Array2D_Double Jk, 
   const Rox_Point2D_Double pts, 
   const Rox_Uint nbpts
);

//! Compute the jacobian for stereo camera calibration (f, cu, cv) wrt 2d point list
//! \param  [out]  Jk             The Jacobian
//! \param  [in ]  pts            List of 2D points
//! \param  [in ]  nbpts          Length of the 2D point list
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_perspective_stereo_calibration_f_cu_cv (
   Rox_Array2D_Double Jk, 
   Rox_Point2D_Double pts, 
   Rox_Uint nbpts
);

//! Compute the jacobian for camera pose
//! \param  [out]  JT             The Jacobian
//! \param  [in ]  K              The camera instrinsic parameters
//! \param  [in ]  pose           The camera pose
//! \param  [in ]  pts            3D points of the calibration template
//! \param  [in ]  pc             2D points of the detected template
//! \param  [in ]  zc             Depth of each model point
//! \param  [in ]  nbpts          Length of the point lists
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_perspective_stereo_calibration_pose (
   Rox_Array2D_Double JT, 
   Rox_Array2D_Double K, 
   Rox_Array2D_Double pose, 
   Rox_Point3D_Double pts, 
   Rox_Point2D_Double pc, 
   Rox_Double *zc, 
   Rox_Uint nbpts
);

//! Compute the jacobian for the stereo pose
//! \param  [out]  JT             The Jacobian
//! \param  [in ]  K              The camera instrinsic parameters
//! \param  [in ]  rTl            The stereo pose
//! \param  [in ]  lTo            The left camera pose wrt to the template
//! \param  [in ]  rTo            The right camera pose wrt to the template
//! \param  [in ]  pts            3D points of the calibration template
//! \param  [in ]  nbpts          Length of the point lists
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_perspective_stereo_calibration_pose_intercamera (
   Rox_Array2D_Double JT, 
   Rox_Array2D_Double K, 
   Rox_Array2D_Double rTl, 
   Rox_Array2D_Double lTo, 
   Rox_Array2D_Double rTo, 
   Rox_Point3D_Double pts, 
   Rox_Uint nbpts
);

//! @} 

#endif // __OPENROX_PERSPECTIVE_STEREO_CALIBRATION__
