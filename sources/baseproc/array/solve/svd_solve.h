//==============================================================================
//
//    OPENROX   : File svd_solve.h
//
//    Contents  : API of svd_solve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SVD_SOLVE__
#define __OPENROX_SVD_SOLVE__

#include <generated/array2d_double.h>

//! \ingroup Linalg
//! \addtogroup solve
//! @{

//! Solve a linear system A*x = b using the SVD decomposition: x = pinv(A) * b
//! \param  [out]  x 	          The solution vector
//! \param  [in ]  A 	          The input matrix
//! \param  [in ]  b 	          The input vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_svd_solve(Rox_Array2D_Double x, const Rox_Array2D_Double A, const Rox_Array2D_Double b);

//! Solve a linear system A*x = 0 using the SVD decomposition
//! \param  [out]  x              The solution vector
//! \param  [in ]  A              The input matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_svd_solve_homogeneous_system(Rox_Array2D_Double x, const Rox_Array2D_Double A);

//! @} 

#endif
