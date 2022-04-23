//==============================================================================
//
//    OPENROX   : File roxrgba_to_roxgray.h
//
//  	Contents  : API of roxrgba_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROXRGBA_TO_ROXGRAY__
#define __OPENROX_IMAGE_ROXRGBA_TO_ROXGRAY__

#include <baseproc/image/image.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a rgba rox internal image to grayscale rox image
//! \param  [out]  image_gray     Destination grayscale image
//! \param  [in ]  image_rgba     Source rgba image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxrgba_to_roxgray ( Rox_Image image_gray, const Rox_Image_RGBA image_rgba );

//! Convert a rgba rox internal image to grayscale rox image
//! \param  [out]  image_gray     Destination grayscale image
//! \param  [in ]  image_rgba     Source rgba image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxrgba_to_roxgray_approx ( Rox_Image image_gray, const Rox_Image_RGBA image_rgba );

//! Convert a rgba rox internal image to grayscale rox image
//! \param  [out]  image_gray     Destination grayscale image
//! \param  [in ]  image_rgba     Source rgba image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxrgba_flip_to_roxgray_approx ( Rox_Image image_gray, const Rox_Image_RGBA image_rgba );

//! @} 

#endif // __OPENROX_IMAGE_ROXRGBA_TO_ROXGRAY__
