//==============================================================================
//
//    OPENROX   : File rgb_to_roxgray.h
//
//    Contents  : API of rgb_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_RGB_TO_ROXGRAY__
#define __OPENROX_IMAGE_RGB_TO_ROXGRAY__

#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a RGB buffer to grayscale image
//! \param [out] dest destination image
//! \param [in] buffer source image
//! \param [in] stride input buffer stride
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_rgb_to_roxgray(Rox_Image dest, const Rox_Uchar * buffer, const Rox_Sint stride);

//! @}

#endif // __OPENROX_IMAGE_RGB_TO_ROXGRAY__
