//==============================================================================
//
//    OPENROX   : File rgb_to_roxrgba.h
//
//    Contents  : API of rgb_to_roxrgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_RGB_TO_ROXRGBA__
#define __OPENROX_IMAGE_RGB_TO_ROXRGBA__

#include <baseproc/image/image_rgba.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a RGB buffer to rox color image
//! \param  [out]  image_rgba     destination image
//! \param  [in ]  buffer         source buffer
//! \param  [in ]  stride         input buffer stride
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_rgb_to_roxrgba(Rox_Image_RGBA image_rgba, const Rox_Uchar * buffer, const Rox_Sint stride);

//! @} 

#endif // __OPENROX_IMAGE_RGB_TO_ROXRGBA__
