//==============================================================================
//
//    OPENROX   : File inverse_lu.h
//
//    Contents  : API of inverse_lu module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INVERSE_LU__
#define __OPENROX_INVERSE_LU__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Compute the inverse of a matrix using LU decompostion
//! \param  [out]  Mi          The inversed matrix
//! \param  [in ]  M           The matrix to inverse
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_inverse_lu ( Rox_Array2D_Double Mi, const Rox_Array2D_Double M );

//! @}

#endif
