//==============================================================================
//
//    OPENROX   : File mat3x3inv.h
//
//    Contents  : API of mat3x3inv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MAT3X3_INVERSE__
#define __OPENROX_MAT3X3_INVERSE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Given a 3x3 real matrix A compute A^-1.
//! \param  [out]  output         The 3x3 result inverse
//! \param  [in ]  matrix         The 3x3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_mat3x3_inverse ( Rox_Array2D_Double output, const Rox_Array2D_Double matrix );

//! @} 

#endif // __OPENROX_MAT3X3_INVERSE__
