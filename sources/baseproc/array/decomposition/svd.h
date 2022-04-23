//==============================================================================
//
//    OPENROX   : File svd.h
//
//    Contents  : API of svd module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SVD__
#define __OPENROX_SVD__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \defgroup SVD SVD
//! \brief Singular Values Decomposition.

//! \addtogroup SVD
//! @{

//! Given a generic matrix "input" of size (rows x cols), compute the svd U, S, V such that input = U * S * V^T
//! \param [out] U      The left special result matrix (input rows * input rows)
//! \param [out] S      The middle special result vector (cols = 1, rows=min(input rows, input cols))
//! \param [out] V      The right special result matrix (input cols * input cols)
//! \param [in] input   The matrix to decompose
//! \warning Must be rows > cols, and the singular values are not sorted
//! \warning The matrix U is not fully computed (U must be rows x rows and it is filled with 0 in the last rows-cols columns)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_svd(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V, const Rox_Array2D_Double input);

//! Given a generic matrix A=QR, compute the svd U,S,V such that A=Q*R=U*S*V^T
//! \param [out] U      The left special result matrix (input rows * input rows)
//! \param [out] S      The middle special result vector (cols = 1, rows=min(input rows, input cols))
//! \param [out] V      The right special result matrix (input cols * input cols)
//! \param [in]  Q      The orthogonal input matrix given by QR pivoting decomposition
//! \param [in]  R      The upper triangular input matrix given by QR pivoting decomposition
//! \param [in]  P      The permutation input matrix given by QR pivoting decomposition
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_svd_jacobi(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V, const Rox_Array2D_Double Q, const Rox_Array2D_Double R, const Rox_Array2D_Double P);

//! @}

#endif
