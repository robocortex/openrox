//==============================================================================
//
//    OPENROX   : File covariance_backpropagation_se3.h
//
//    Contents  : API of covariance_backpropagation_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_COVARIANCE_SE3_POINTS__
#define __OPENROX_COVARIANCE_SE3_POINTS__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>


//!  \ingroup Maths
//!  \defgroup

//!  \addtogroup
//!  @{


//! Covariance backpropagation, following "Multiple View Geometry" chapter 5
//! E_pose = pinv( sum( J'. E_meas^{-1} . J ) )
//!
//! \param [out] E          : pose covariance matrix with columns ordered as ( R_yz, R_zx, R_xy, tx, ty, tz )
//! \param [in]  cTo        : the pose input
//! \param [in]  K          : the camera calibration
//! \param [in]  measures   : the 2D observed pixels (in pixels)
//! \param [in]  references : the 3D reference pixels
//!
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_covariance_se3_points ( 
   Rox_Matrix               E,
   Rox_MatSE3               cTo,
   Rox_MatUT3               K,
   Rox_DynVec_Point2D_Double measures,
   Rox_DynVec_Point3D_Double references 
);

//! @}

#endif
