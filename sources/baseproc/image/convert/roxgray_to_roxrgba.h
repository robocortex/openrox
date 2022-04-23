//==============================================================================
//
//    OPENROX   : File roxgray_to_roxrgba.h
//
//    Contents  : API of roxgray_to_roxrgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROXGRAY_TO_ROXRGBA__
#define __OPENROX_IMAGE_ROXGRAY_TO_ROXRGBA__

#include <baseproc/image/image.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a grayscale rox internal image to rgba rox image
//! r = gray; g = gray; b = gray; a = 255;
//! \param  [out]  image_rgba     destination image
//! \param  [in ]  image_gray     source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxgray_to_roxrgba ( Rox_Image_RGBA image_rgba, const Rox_Image image_gray);

//! @}

#endif // __OPENROX_IMAGE_ROXGRAY_TO_ROXRGBA__
