//==============================================================================
//
//    OPENROX   : File point2d_undistort.h
//
//    Contents  : API of pointundistort module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT2D_UNDISTORT__
#define __OPENROX_POINT2D_UNDISTORT__

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matut3.h>

//!   \ingroup Geometry
//!   \addtogroup Point2D_Undistort 
//!   @{

//!   Try to undistort a point. As it is a non inversible function, this is only an iterative approximation.
//!   \param  [out]   undistorted    The undistorted coordinates of the point (result in meters)
//!   \param  [in]    distorted      The distorted coordinates of the point
//!   \param  [in]    calib          A 3x3 camera calibration matrix with focal, skew and central point.
//!   \param  [in]    distortion     A 5x1 array [k1, k2, k3, k4, k5] with 3 radial distortion parameters [k1,k2,k5] and 2 tangential distortion parameters [k3,k4] as in Bouguet's matlab calibration toolbox.
//!   \return An error code
//!   \see http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
//!   \warning replace this method with the Robocortex method implemented in "rox_mat/modules/vision/calibration/distortion/point_undistort.m"
//!   \todo To be tested

ROX_API Rox_ErrorCode rox_point2d_double_undistort (
   Rox_Point2D_Double undistorted, 
   Rox_Point2D_Double distorted, 
   Rox_MatUT3 calib, 
   Rox_Array2D_Double distortion
);

//!   @} 

#endif // __OPENROX_POINT2D_UNDISTORT__
