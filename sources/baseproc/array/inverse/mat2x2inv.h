//==============================================================================
//
//    OPENROX   : File mat2x2inv.h
//
//    Contents  : API of mat2x2inv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MAT2X2_SOLVE__
#define __OPENROX_MAT2X2_SOLVE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Given a 2x2 real matrix A compute A^-1.
//! \param  [out]  output the 2x2 result inverse
//! \param  [in ]  matrix the 2x2 matrix
//! \return An error code.
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_mat2x2_inverse ( Rox_Array2D_Double output, Rox_Array2D_Double matrix );

//! @} 

#endif
