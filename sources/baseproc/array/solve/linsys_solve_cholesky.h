//==============================================================================
//
//    OPENROX   : File linsys_solve_cholesky.h
//
//    Contents  : API of linsys_solve_cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_SOLVE_CHOLESKY__
#define __OPENROX_LINSYS_SOLVE_CHOLESKY__

#include <generated/array2d_double.h>

//! \ingroup Linalg
//! \addtogroup solve
//! @{

//! Solve a symmetric linear system S * x = v using the colesky decomposition: x = cholesky_inv( S ) * v
//! \param  [out]  x 	          The solution vector
//! \param  [in ]  S 	          The input symmetric matrix
//! \param  [in ]  v 	          The input vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linsys_solve_cholesky ( Rox_Array2D_Double x, const Rox_Array2D_Double A, const Rox_Array2D_Double b);

ROX_API int rox_ansi_array_float_linsys_solve_cholesky ( float * x, float * S, int S_size, float * v );

//! @} 

#endif
