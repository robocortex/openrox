//==============================================================================
//
//    OPENROX   : File linsys_tutvsusv_light_affine_texture_model2d.h
//
//    Contents  : API of linsys_tutvsusv_light_affine_texture_model2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_JACOBIANTUTVSUSV_LA__
#define __OPENROX_JACOBIANTUTVSUSV_LA__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \defgroup tutvsusvjacobians tutvsusvjacobians

//! \addtogroup tutvsusvjacobians
//! @{

//! Compute the jacobian for image wrt tutvsusv,alpha,beta
//! \param  [out]  LtL            The result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte            The result projected vector (J^t*diff)
//! \param  [in ]  mean           The Refernce plus current image mean
//! \param  [in ]  diff           The error image
//! \param  [in ]  gradient_x     The luminance gradient in x vector
//! \param  [in ]  gradient_y     The luminance gradient in y vector
//! \param  [in ]  input_mask     The input mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_tutvsusv_light_affine_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Array2D_Float gradient_x, 
   const Rox_Array2D_Float gradient_y, 
   const Rox_Imask input_mask
);

//! @} 

#endif
