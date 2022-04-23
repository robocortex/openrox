//==============================================================================
//
//    OPENROX   : File mulmatmat.h
//
//    Contents  : API of mulmatmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MULMATMAT__
#define __OPENROX_MULMATMAT__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Transpose
//! @{

//! \ingroup Array2D_Double
//! \brief Classic matrix wise matrix multiplication
//! \param  [out] res               The multiplied matrix (rows = one.rows, cols = two.cols)
//! \param  [in]  one               The left operand
//! \param  [in]  two               The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_mulmatmat(Rox_Array2D_Double res, Rox_Array2D_Double one, Rox_Array2D_Double two);

//! @} 

#endif
