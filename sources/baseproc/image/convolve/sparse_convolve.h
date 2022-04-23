//==============================================================================
//
//    OPENROX   : File sparse_convolve.h
//
//    Contents  : API of sparse_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SPARSE_CONVOLVE__
#define __OPENROX_SPARSE_CONVOLVE__

#include <generated/array2d_float.h>
#include <generated/dynvec_sparse_value.h>

#include <system/errors/errors.h>

#include <baseproc/image/image.h>

//! \addtogroup Convolution
//! @{

//! A generic 2D array convolution with sparse kernels
//! \param  [out]  dest           Convolved result, undefined on image borders
//! \param  [in ]  source         The array2D to convolve
//! \param  [in ]  kernel         The kernel to apply
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_convolve_with_sparse_kernel (
   Rox_Array2D_Float dest, 
   Rox_Array2D_Float source, 
   Rox_DynVec_Sparse_Value kernel);

//! @} 

#endif // __OPENROX_SPARSE_CONVOLVE__
