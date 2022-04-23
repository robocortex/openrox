//==============================================================================
//
//    OPENROX   : File lotinverse.h
//
//    Contents  : API of lotinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LOT_INVERSE__
#define __OPENROX_LOT_INVERSE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Compute the inverse of a symmetric positive definite matrix
//! \param  [out]  dest           The inversed matrix
//! \param  [in ]  input          The symmetric positive definite matrix to inverse
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_lotinverse ( Rox_Array2D_Double dest, const Rox_Array2D_Double input );

//! @}

#endif
