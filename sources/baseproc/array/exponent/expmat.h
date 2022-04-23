//==============================================================================
//
//    OPENROX   : File expmat.h
//
//    Contents  : API of expmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EXPMAT__
#define __OPENROX_EXPMAT__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup matrixexponent
//! @{

//! Pade approximation to the matrix exponential
//! \param [out] dest the exponential
//! \param [in] input the input array
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_expmat(Rox_Array2D_Double dest, Rox_Array2D_Double input);

//! @}

#endif
