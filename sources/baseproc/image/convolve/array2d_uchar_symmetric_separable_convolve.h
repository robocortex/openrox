//============================================================================
//
//    OPENROX   : File symm_convolve.h
//
//    Contents  : API of symm_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ARRAY2D_UCHAR_SYMMETRIC_SEPARABLE_CONVOLVE__
#define __OPENROX_ARRAY2D_UCHAR_SYMMETRIC_SEPARABLE_CONVOLVE__

#include <system/errors/errors.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <baseproc/image/image.h>

//! \addtogroup Convolution
//! @{

//! Performs a convolution given that the kernel is symmetric in all directions (separable and symmetric)
//! \param  [out]  output         The processed output
//! \param  [in ]  input          The input array to convolve
//! \param  [in ]  kernel         The convolution operator (1 row, odd columns count)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_symmetric_seperable_convolve (
   Rox_Image output, 
   const Rox_Image input, 
   const Rox_Array2D_Float kernel
);

//! @} 

#endif
