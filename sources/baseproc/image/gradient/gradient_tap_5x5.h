//==============================================================================
//
//    OPENROX   : File gradient_tap_5x5.h
//
//    Contents  : API of gradientsobel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_GRADIENT_TAP_5X5__
#define __OPENROX_GRADIENT_TAP_5X5__

#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>

//! Compute gradient using Farid and Simoncelli's symmetric 5 tap kernel
//! The kernels of fixed size 5 x 5 is optimized so that the Fourier transform approximates their correct derivative relationship.
//! \param  [out] gx          The result horizontal gradient
//! \param  [out] gy          The result vertical gradient
//! \param  [in]  source      The source image (M*N)
//! \param  [in]  buffer      The buffer of size (1*max(M,N))
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_gradient5 (
   Rox_Array2D_Float gx, 
   Rox_Array2D_Float gy, 
   Rox_Array2D_Float source, 
   Rox_Array2D_Float buffer
);


#endif
