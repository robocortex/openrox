//==============================================================================
//
//    OPENROX   : File qr.h
//
//    Contents  : API of qr module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QR__
#define __OPENROX_QR__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \defgroup QR QR
//! \brief QR Decomposition.

//! \addtogroup QR
//! @{

//! Given a generic matrix A, compute the QR Decomposition Q,R such that A = Q * R
//! \param  [out] Q           The orthogonal matrix (input rows , input rows)
//! \param  [out] R           The upper triangular matrix (input row , input cols)
//! \param  [in]  A           The matrix to decompose 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_qr(Rox_Array2D_Double Q, Rox_Array2D_Double R, const Rox_Array2D_Double A);

//! Given a generic matrix A, compute the pivoting QR Decomposition Q,R,P such that A = Q*R*P
//! \param  [out] Q           The orthogonal matrix (input rows , input rows)
//! \param  [out] R           The upper triangular matrix (input row , input cols)
//! \param  [out] P           The permutation matrix (input cols , input cols)
//! \param  [in]  A           The matrix to decompose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_qrp(Rox_Array2D_Double Q, Rox_Array2D_Double R, Rox_Array2D_Double P, const Rox_Array2D_Double A);

//! @} 

#endif
