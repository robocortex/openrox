//==============================================================================
//
//    OPENROX   : File cholesky.h
//
//    Contents  : API of cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CHOLESKY__
#define __OPENROX_CHOLESKY__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \defgroup Cholesky Cholesky
//! \brief Cholesky Decomposition.

//! \addtogroup Cholesky Cholesky
//! @{

//! Compute the cholesky decomposition of a symmetric positive definite matrix S = L * L^T 
//! \param  [out]  L              The resulting lower triangular matrix
//! \param  [in ]  S              The symmetric positive definite matrix to decompose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_cholesky_decomposition (
   Rox_Array2D_Double L, 
   const Rox_Array2D_Double S
);

//! @} 

#endif
