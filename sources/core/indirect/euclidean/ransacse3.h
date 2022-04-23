//==============================================================================
//
//    OPENROX   : File ransacse3.h
//
//    Contents  : API of ransacse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANSAC_SE3__
#define __OPENROX_RANSAC_SE3__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_sint.h>



//! \ingroup MatSE3
//! \addtogroup RANSACSE3
//! @{


//! Robustly estimate pose from points in two views using ransac algorithm (fischer 1981)
//!
//! \param  [out]  pose           Computed pose
//! \param  [out]  inliers2D      Destination points which are not violated by the model
//! \param  [out]  inliers3D      Source points which are not violated by the model
//! \param  [in ]  cur2D          2D coordinates (pixels) of destination points
//! \param  [in ]  ref3D          3D coordinates (meters) of source points
//! \param  [in ]  calib          Camera calibration
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ransac_p3p (
   Rox_Array2D_Double       pose,
   Rox_DynVec_Point2D_Float inliers2D,
   Rox_DynVec_Point3D_Float inliers3D,
   Rox_DynVec_Point2D_Float cur2D,
   Rox_DynVec_Point3D_Float ref3D,
   Rox_Array2D_Double       calib 
);


//! Robustly estimate pose from points in two views using ransac algorithm (fischer 1981)
//!
//! \param  [out]  pose           Computed pose
//! \param  [out]  inliers2D      Destination points which are not violated by the model
//! \param  [out]  inliers3D      Source points which are not violated by the model
//! \param  [in ]  cur2D          2D coordinates (pixels) of destination points
//! \param  [in ]  ref3D          3D coordinates (meters) of source points
//! \param  [in ]  calib          Camera calibration
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ransac_p3p_double (
   Rox_Array2D_Double        pose,
   Rox_DynVec_Point2D_Double inliers2D,
   Rox_DynVec_Point3D_Double inliers3D,
   Rox_DynVec_Point2D_Double cur2D,
   Rox_DynVec_Point3D_Double ref3D,
   Rox_Array2D_Double        calib 
);


//! @}

#endif
