//==============================================================================
//
//    OPENROX   : File canny.h
//
//    Contents  : API of canny module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CANNY__
#define __OPENROX_CANNY__

#include <baseproc/image/image.h>

//! \addtogroup Edges
//! @{

//! Compute the canny response for an image
//! \param  [out]  result         the result image with binary edges
//! \param  [in ]  source         the input image to detect edges into
//! \param  [in ]  sigma          the smoothing sigma
//! \param  [in ]  low            the low threshold
//! \param  [in ]  high           the high treshold
//! \return An error code
ROX_API Rox_ErrorCode rox_canny_process (
   Rox_Image result, 
   const Rox_Image source, 
   const Rox_Double sigma, 
   const Rox_Float low, 
   const Rox_Float high
);

//! @}

#endif // __OPENROX_CANNY__
