//==============================================================================
//
//    OPENROX   : File yuv422_to_roxgray.h
//
//    Contents  : API of yuv422_to_roxgray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_YUV422_TO_ROXGRAY__
#define __OPENROX_IMAGE_YUV422_TO_ROXGRAY__

#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a YUV422 buffer to grayscale image
//! \param  [out] dest destination image
//! \param  [in ] buffer source image
//! \param  [in ] stride input buffer stride
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_yuv422_to_roxgray ( Rox_Image dest, const Rox_Uchar * buffer, const Rox_Uint stride);

//! @} 

#endif // __OPENROX_IMAGE_YUV422_TO_ROXGRAY__
