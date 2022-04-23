//=============================================================================
//
//    OPENROX   : File basic_convolve.h
//
//    Contents  : API of basic_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#ifndef __OPENROX_BASIC_CONVOLVE__
#define __OPENROX_BASIC_CONVOLVE__

#include <system/errors/errors.h>
#include <generated/array2d_float.h>

//! \ingroup Filtering
//! \defgroup Convolution Convolution

//! \addtogroup Convolution
//! @{

//! A generic 2D array convolution function to test kernels
//! \param  [out] dest              Convolved result, undefined on image borders
//! \param  [in]  source            The array2D to convolve
//! \param  [in]  kernel            The kernel to apply
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_convolve_with_kernel(Rox_Array2D_Float dest, Rox_Array2D_Float source, Rox_Array2D_Float kernel);

//! @} 

#endif
