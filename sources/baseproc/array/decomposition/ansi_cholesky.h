//==============================================================================
//
//    OPENROX   : File ansi_cholesky.h
//
//    Contents  : API of ansi_cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_CHOLESKY__
#define __OPENROX_ANSI_CHOLESKY__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \ingroup  Linalg
//! \defgroup Cholesky Cholesky
//! \brief Cholesky Decomposition.

//! \addtogroup Cholesky Cholesky
//! @{

//! Compute the cholesky decomposition of a symmetric positive definite matrix S = L * L^T 
//! \param  [in, out]   L              The input symmetrix matrix and the output lower triangular matrix
//! \param  [in ]       L_size         The symmetric positive definite matrix to decompose
//! \return An error code
//! \todo   To be tested
ROX_API int rox_ansi_array_float_cholesky_decomposition_inplace ( float * L_data, int L_size );

ROX_API int rox_ansi_array_double_cholesky_decomposition_inplace ( double * L_data, int L_size );

//! @} 

#endif // __OPENROX_ANSI_CHOLESKY__
