//==============================================================================
//
//    OPENROX   : File interaction_matsl3_texture.h
//
//    Contents  : API of interaction_matsl3_texture module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERACTION_MATSL3_TEXTURE__
#define __OPENROX_INTERACTION_MATSL3_TEXTURE__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup sl3jacobians
//! @{

//! Compute the jacobian for image wrt sl3,alpha,beta
//! \param  [out]  LtL           The 8x8 matrix 
//! \param  [out]  Lte           The 8x1 vector
//! \param  [in ]  Ia            The image average    Ia = (Ir - Iw)/2
//! \param  [in ]  Id            The image difference Id =  Ir - Iw
//! \param  [in ]  Iu            The image gradient along the u axis
//! \param  [in ]  Iv            The image gradient along the v axis
//! \param  [in ]  Im            The image mask
//! \return An error code
//! \todo   To be tested
//! \todo   Rename the function "rox_linsys_matsl3_light_affine_texture"
ROX_API Rox_ErrorCode linsys_texture_matsl3_light_affine (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Imask Im
);

//! @} 

#endif //__OPENROX_INTERACTION_MATSL3_TEXTURE__
