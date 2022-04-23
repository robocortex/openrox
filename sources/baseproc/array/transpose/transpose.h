//==============================================================================
//
//    OPENROX   : File transpose.h
//
//    Contents  : API of transpose module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRANSPOSE__
#define __OPENROX_TRANSPOSE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Transpose
//! @{

//! Classic matrix transpose
//! \param [out] res the transposed matrix (rows = input.cols, cols = input.rows)
//! \param [in] input the input array
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_transpose(Rox_Array2D_Double res, Rox_Array2D_Double input);

//! @} 

#endif
