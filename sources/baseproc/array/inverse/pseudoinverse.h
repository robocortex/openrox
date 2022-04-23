//==============================================================================
//
//    OPENROX   : File pseudoinverse.h
//
//    Contents  : API of pseudoinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PSEUDO_INVERSE__
#define __OPENROX_PSEUDO_INVERSE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Compute the pseudo inverse of a general matrix (M+ = inv(M'*M)*M')
//! \param  [out]  output           The pseudo inversed matrix (M+), expected cols == in_rows and rows == in_cols
//! \param  [in ]  input            The general matrix (M)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_pseudoinverse(Rox_Array2D_Double output, Rox_Array2D_Double input);

//! @} 

#endif // __OPENROX_PSEUDO_INVERSE__
