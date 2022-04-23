//==============================================================================
//
//    OPENROX   : File symm3x3solve.h
//
//    Contents  : API of symm3x3solve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SYMM3X3_SOLVE__
#define __OPENROX_SYMM3X3_SOLVE__

#include <generated/array2d_double.h>

//! \ingroup Linalg
//! \addtogroup solve
//! @{

//! Given a 3x3 symmetric real matrix S and a 3x1 vector v, compute A^-1*v solution of the equation S*x = v
//! \param  [out]  output         The 3x1 result vector
//! \param  [in ]  matrix         The 3x3 symmetric matrix (lower triangular part is used)
//! \param  [in ]  vec            The 3x1 input vector.
//! \return An error code.
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_symm3x3_solve(Rox_Array2D_Double output, Rox_Array2D_Double matrix, Rox_Array2D_Double vec);

//! @}

#endif
