//==============================================================================
//
//    OPENROX   : File linsys_matso3_light_affine_texture.h
//
//    Contents  : API of linsys_matso3_light_affine_texture module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SO3LIGHTAFFINEPREMUL__
#define __OPENROX_SO3LIGHTAFFINEPREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \defgroup so3jacobians so3jacobians
//! \brief Jacobians relative to the SO3 group.

//! \addtogroup so3jacobians
//! @{

//! Compute the jacobian for image wrt so3, alpha, beta with depth fixed to 1
//! \param  [out]  LtL            The result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte            The result projected vector (J^t*diff)
//! \param  [in ]  gx             The luminance gradient in x vector
//! \param  [in ]  gy             The luminance gradient in y vector
//! \param  [in ]  mean           The Reference plus current image mean
//! \param  [in ]  diff           The error image
//! \param  [in ]  mask           The valid pixel image
//! \param  [in ]  pose           The camera pose (in SE(3))
//! \param  [in ]  calib_input    The camera calibration
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_so3_light_affine_premul (
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

//! Compute the jacobian for image wrt so3, alpha, beta with depth fixed to 1
//! \param  [out]  LtL            The result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte            The result projected vector (J^t*diff)
//! \param  [in ]  gx             The luminance gradient in x vector
//! \param  [in ]  gy             The luminance gradient in y vector
//! \param  [in ]  mean           The reference plus current image mean
//! \param  [in ]  diff           The error image
//! \param  [in ]  mask           The valid pixel image
//! \param  [in ]  calib_input    The camera calibration
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_jacobian_so3_simple_light_affine_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float gx, 
   const Rox_Array2D_Float gy, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Imask mask, 
   const Rox_MatUT3 calib_input
);

//! @} 

#endif
