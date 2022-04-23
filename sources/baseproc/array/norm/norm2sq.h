//==============================================================================
//
//    OPENROX   : File norm2sq.h
//
//    Contents  : API of vector norm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VECTOR_NORM__
#define __OPENROX_VECTOR_NORM__

#include <generated/array2d_double.h>

//! \ingroup Linalg
//! \defgroup Vector Vector

//! \ingroup Vector
//! \addtogroup Norm
//! @{

//! Compute the L2 norm of a vector
//! \param [out]  norm     The computed norm
//! \param [in]   vector   The only operand (cols = 1)
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_norm2(Rox_Double * norm2, Rox_Array2D_Double vector);

//! Compute the L2 squared norm of a vector
//! \param [out]  norm     The computed squared norm
//! \param [in]   vector   The only operand (cols = 1)
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_norm2sq(Rox_Double * norm2sq, Rox_Array2D_Double vector);

//! Compute the Frobenius norm of a matrix
//! Note that for a rank-1 matrix, e.g. a vector, the Frobenius norm is equal to the L2 norm 
//! \param [out]  norm     The computed squared norm
//! \param [in]   matrix   The input matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_norm_frobenius(Rox_Double * norm_frobenius, Rox_Array2D_Double matrix);

//! @} 

#endif
