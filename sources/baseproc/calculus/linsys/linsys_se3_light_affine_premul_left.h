//==============================================================================
//
//    OPENROX   : File linsys_se3_light_affine_premul_left.h
//
//    Contents  : API of linsys_se3_light_affine_premul_left module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_SE3_LIGHT_AFFINE_PREMUL_LEFT__
#define __OPENROX_LINSYS_SE3_LIGHT_AFFINE_PREMUL_LEFT__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup se3jacobians
//! @{

//! Compute the jacobian for image wrt se3, alpha, beta with depth fixed to 1
//! Supposing a left pose update. This is needed for odometry with multiple planes
//! \param  [out]  LtL            The result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte            The result projected vector (J^t*diff)
//! \param  [in ]  gx             The luminance gradient in x vector
//! \param  [in ]  gy             The luminance gradient in y vector
//! \param  [in ]  mean           The Refernce plus current image mean
//! \param  [in ]  diff           The error image
//! \param  [in ]  mask           The valid pixel image
//! \param  [in ]  pose           The camera pose
//! \param  [in ]  calib_input    The camera calibration
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_light_affine_premul_left (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float gx, 
   const Rox_Array2D_Float gy, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Imask mask, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calib_input
);

//! @} 

#endif
