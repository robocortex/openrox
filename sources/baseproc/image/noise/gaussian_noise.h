//============================================================================
//
//    OPENROX   : File gaussian_noise.h
//
//    Contents  : API of gaussian_noise module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_GAUSSIAN_NOISE__
#define __OPENROX_GAUSSIAN_NOISE__

#include <system/errors/errors.h>
#include <baseproc/image/image.h>

//! \addtogroup Image
//! @{

//! Apply a gaussian additive noise to an image
//! \param  [out]  dst            The noisy image result
//! \param  [in ]  src            The input image to add noise to
//! \param  [in ]  sigma          The sigma of the gaussian
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_gaussian_noise (
   Rox_Image dst, 
   const Rox_Image src, 
   const Rox_Double sigma);

//! @} 

#endif
