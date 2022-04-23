//==============================================================================
//
//    OPENROX   : File vvs_points_se3.h
//
//    Contents  : API of vvs_points_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VVS_POINTS_SE3__
#define __OPENROX_VVS_POINTS_SE3__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>


//!  \ingroup Maths
//!  \defgroup Optimization Optimization

//!  \ingroup Optimization
//!  \defgroup Servoing Visual Servoing

//!  \addtogroup Servoing
//!  @{


//! Given a list 3D points and associated 2D points measurements, minimize the reprojection error by looking for a valid pose.
//! Visual virtual servoing approach (linear approx.)
//! \param  [out]  pose           the pose result and input
//! \param  [in ]  calib          the camera calibration
//! \param  [in ]  measures       the 2D observed pixels (in pixels)
//! \param  [in ]  references     the 3D reference pixels
//! \param  [in ]  maxdist_prefilter maximum distance for a point before it is prefiltered, in pixel units
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_float_refine_pose_vvs (
   Rox_Array2D_Double pose, 
   Rox_Array2D_Double calib, 
   Rox_DynVec_Point2D_Float measures, 
   Rox_DynVec_Point3D_Float references, 
   Rox_Double maxdist_prefilter
);


//! Given a list 3D points and associated 2D points measurements, minimize the reprojection error by looking for a valid pose.
//! Visual virtual servoing approach (linear approx.)
//! \param  [out]  pose          the pose result and input
//! \param  [in ]  calib         the camera calibration
//! \param  [in ]  measures      the 2D observed pixels (in pixels)
//! \param  [in ]  references    the 3D reference pixels
//! \param  [in ]  maxdist_prefilter maximum distance for a point before it is prefiltered, in pixel units
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_double_refine_pose_vvs (
   Rox_Array2D_Double pose, 
   Rox_Array2D_Double calib, 
   Rox_DynVec_Point2D_Double measures, 
   Rox_DynVec_Point3D_Double references, 
   Rox_Double maxdist_prefilter
);

//! This function must be placed in a different module like for example "vvs_tools.c"
//! \param [out] ret_score
//! \param [in]  weight
//! \param [in]  verr
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_vvs_score (
   Rox_Double * ret_score, 
   Rox_Array2D_Double weight, 
   Rox_Array2D_Double verr
);


//! @}

#endif
