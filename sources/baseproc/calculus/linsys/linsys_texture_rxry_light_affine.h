//==============================================================================
//
//    OPENROX   : File linsys_rxry_light_affine_texture.h
//
//    Contents  : API of linsys_rxry_light_affine_texture module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_JACOBIANR2LAPREMUL__
#define __OPENROX_JACOBIANR2LAPREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \addtogroup Jacobians
//! @{

//! Jacobian for image wrt rx, ry, alpha
//! The suffix _premul means that the linear system in normal form is computed
//! \param  [out]  LtL            The result L'*L matrix
//! \param  [out]  Lte            The result L'* e vector
//! \param  [in ]  diffs          The input measured error
//! \param  [in ]  gradient_x     The x gradient patch
//! \param  [in ]  gradient_y     The y gradient patch
//! \param  [in ]  input_mask     The patch mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode linsys_texture_rxry_light_affine (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float diffs, 
   const Rox_Array2D_Float gradient_x, 
   const Rox_Array2D_Float gradient_y, 
   const Rox_Imask input_mask
);

//! @} 

#endif
