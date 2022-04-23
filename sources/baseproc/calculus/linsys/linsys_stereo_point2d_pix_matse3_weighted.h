//==============================================================================
//
//    OPENROX   : File linsys_stereo_point2d_pix_matse3_weighted.h
//
//    Contents  : API of linsys_stereo_point2d_pix_matse3_weighted module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_STEREO_POINT2D_PIX_MATSE3_WEIGHTED__
#define __OPENROX_LINSYS_STEREO_POINT2D_PIX_MATSE3_WEIGHTED__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \addtogroup se3jacobians
//! @{
 
//! Compute the jacobian for points wrt se3,alpha,beta
//! \param  [out]  LtL            the result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte            the result projected vector (J^t*diff)
//! \param  [in ]  diff           the error vector
//! \param  [in ]  weight         the weight vector (for robust estimation)
//! \param  [in ]  meters         the points in meters which were used to compute the error
//! \param  [in ]  calib          calibration matrix
//! \param  [in ]  pose           stereo head pose
//! \param  [in ]  rTl            relative pose of the camera wrt the head
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_from_stereo_points_pixels_weighted_premul_float (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Double diff, 
   const Rox_Array2D_Double weight, 
   const Rox_DynVec_Point3D_Float meters, 
   const Rox_Array2D_Double calib, 
   const Rox_Array2D_Double pose, 
   const Rox_Array2D_Double rTl
);

//! @}

#endif
