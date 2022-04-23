//==============================================================================
//
//    OPENROX   : File tridiagonal.h
//
//    Contents  : API of tridiagonal module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRIDIAGONAL__
#define __OPENROX_TRIDIAGONAL__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Tridiagonal
//! @{

//! Compute a tridiagonal matrix form input matrix
//! The tridiagonal matrix has the same eigenvalues of the input matrix
//! \param  [out] T              The tridiagonal matrix (rows = input.cols, cols = input.rows)
//! \param  [in]  M              The input array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_tridiagonal(Rox_Array2D_Double T, const Rox_Array2D_Double M);

//! Test if a matrix is tridiagonal 
//! \param  [out] is_tridiagonal    The result of the test
//! \param  [in]  T                 The input array to be tested
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_is_tridiagonal(Rox_Sint * is_tridiagonal, const Rox_Array2D_Double T);

//! @} 

#endif
