//==============================================================================
//
//    OPENROX   : File crossprod.h
//
//    Contents  : API of crossprod module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CROSSPROD__
#define __OPENROX_CROSSPROD__

#include <generated/array2d_double.h>

//! \ingroup Vector
//! \addtogroup Crossprod
//!  @{

//! Compute the vector cross-product of two (3x1) vectors : res = skew(one) * two
//! \param  [out]  res            The result vector (size 3*1)
//! \param  [in ]  one            The left parameter (size 3*1)
//! \param  [in ]  two            The right parameter (size 3*1)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_crossprod (
   Rox_Array2D_Double res, 
   const Rox_Array2D_Double one, 
   const Rox_Array2D_Double two
);

//! @}

#endif
