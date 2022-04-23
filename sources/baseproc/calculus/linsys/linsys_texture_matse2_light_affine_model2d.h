//==============================================================================
//
//    OPENROX   : File linsys_matse2_light_affine_texture_model2d.h
//
//    Contents  : API of linsys_matse2_light_affine_texture_model2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SE2LIGHTAFFINEPREMUL__
#define __OPENROX_SE2LIGHTAFFINEPREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \defgroup SE2jacobians SE2jacobians

//! \addtogroup SE2jacobians
//! @{

//! Compute the jacobian for image wrt se2,alpha,beta with depth fixed to 1
//! \param  [out]  LtL           the result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte           the result projected vector (J^t*diff)
//! \param  [in ]  gx            the luminance gradient in x vector
//! \param  [in ]  gy            the luminance gradient in y vector
//! \param  [in ]  mean          the Reference plus current image mean
//! \param  [in ]  diff          the error image
//! \param  [in ]  mask          the valid pixel image
//! \param  [in ]  pose_se2      the camera pose (in SE(2)
//! \param  [in ]  calib_input   the camera calibration
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_jacobian_se2_light_affine_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float gx, 
   const Rox_Array2D_Float gy, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Imask mask, 
   const Rox_Array2D_Double pose_se2, 
   const Rox_Array2D_Double calib_input
);

//! @} 

#endif
