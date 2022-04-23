//==============================================================================
//
//    OPENROX   : File ransac_se3_vvs_se3.h
//
//    Contents  : API of ransac_se3_vvs_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANSAC_SE3_VVS_SE3__
#define __OPENROX_RANSAC_SE3_VVS_SE3__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_sint.h>


//! \ingroup MatSE3
//! \addtogroup RANSACSE3
//! @{


//! Robustly estimate pose from 3D-2D matches using ransac algorithm (fischer 1981).
//! Then apply Virtual Visual Servoing on the inliers found.
//!
//! \param  [out]  pose           Computed pose
//! \param  [out]  inliers2D      Inlier 2D measures
//! \param  [out]  inliers3D      Inlier 3D points
//! \param  [in ]  input2D        2D measures
//! \param  [in ]  input3D        3D points
//! \param  [in ]  calib          Camera calibration
//! \param  [in ]  maxdist        Maximum distance for a point before it is prefiltered, in pixel units
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ransac_se3_vvs_se3_float(
   Rox_MatSE3               pose,
   Rox_DynVec_Point2D_Float inliers2D,
   Rox_DynVec_Point3D_Float inliers3D,
   Rox_DynVec_Point2D_Float input2D,
   Rox_DynVec_Point3D_Float input3D,
   Rox_MatSL3               calib,
   Rox_Double               maxdist_prefilter
);


//! Robustly estimate pose from 3D-2D matches using ransac algorithm (fischer 1981).
//! Then apply Virtual Visual Servoing on the inliers found.
//!
//! \param  [out] pose      : Computed pose
//! \param  [out] inliers2D : inlier 2D measures
//! \param  [out] inliers3D : inlier 3D points
//! \param  [in ]  input2D   : 2D measures
//! \param  [in ]  input3D   : 3D points
//! \param  [in ]  calib     : Camera calibration
//! \param  [in ]  maxdist   : maximum distance for a point before it is prefiltered, in pixel units
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ransac_se3_vvs_se3_double (
   Rox_MatSE3                pose,
   Rox_DynVec_Point2D_Double inliers2D,
   Rox_DynVec_Point3D_Double inliers3D,
   Rox_DynVec_Point2D_Double input2D,
   Rox_DynVec_Point3D_Double input3D,
   Rox_MatSL3                calib,
   Rox_Double                maxdist_prefilter 
);



//! @}

#endif
