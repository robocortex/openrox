//==============================================================================
//
//    OPENROX   : File yuv422_to_roxrgba.h
//
//    Contents  : API of yuv422_to_roxrgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_YUV422_TO_ROXRGBA__
#define __OPENROX_IMAGE_YUV422_TO_ROXRGBA__

#include <baseproc/image/image_rgba.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a YUV422 buffer to rgba rox image
//! \param  [out]  dest           Destination image
//! \param  [in ]  buffer         Source image
//! \param  [in ]  stride         Input buffer stride
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_yuv422_to_roxrgba(Rox_Image_RGBA dest, const Rox_Uchar * buffer, const Rox_Uint stride);

//! @} 

#endif
