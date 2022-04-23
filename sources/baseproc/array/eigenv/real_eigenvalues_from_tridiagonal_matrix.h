//============================================================================
//
//    OPENROX   : File real_eigenvalues_from_tridiagonal_matrix.h
//
//    Contents  : API of real_eigenvalues_from_tridiagonal_matrix module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ROX_REAL_EIGENVALUES_FROM_TRIDIAGONAL_MATRIX__
#define __OPENROX_ROX_REAL_EIGENVALUES_FROM_TRIDIAGONAL_MATRIX__

#include <generated/array2d_double.h>
#include <generated/dynvec_double.h>

//! \ingroup Eigenv
//! \addtogroup Eigenv
//! @{

//! Compute eigenvalues from a matrix in tridiagonal form
//! \param  [out]  e              The computed eigenvalues 
//! \param  [in ]  T              The matrix in tridiagonal form
//! \return An error code
ROX_API Rox_ErrorCode rox_real_eigenvalues_from_tridiagonal_matrix(Rox_DynVec_Double e, const Rox_Array2D_Double T);

//! @}

#endif // __OPENROX_ROX_REAL_EIGENVALUES_FROM_TRIDIAGONAL_MATRIX__
