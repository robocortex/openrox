//==============================================================================
//
//    OPENROX   : File mulmatmattrans.h
//
//    Contents  : API of mulmatmattrans module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MULMATMATTRANS__
#define __OPENROX_MULMATMATTRANS__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Transpose
//! @{

//! \ingroup  Array2D_Double
//! \brief Classic matrix wise matrix multiplication (Second matrix being transposed
//! \param  [out] res the multiplied matrix (rows = one.rows, cols = two.rows)
//! \param  [in] one the left operand
//! \param  [in] two the right operand
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_mulmatmattrans(Rox_Array2D_Double res, Rox_Array2D_Double one, Rox_Array2D_Double two);

//! @} 

#endif
