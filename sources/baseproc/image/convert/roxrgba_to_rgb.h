//==============================================================================
//
//    OPENROX   : File roxrgba_to_rgb.h
//
//    Contents  : API of roxrgba_to_rgb module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ROXRGBA_TO_RGB__
#define __OPENROX_ROXRGBA_TO_RGB__

#include <baseproc/image/image_rgba.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//!   @{

//! Convert a color image to RGB buffer
//! \param  [out]  buffer         destination buffer, already allocated
//! \param  [in ]  source         source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxrgba_to_rgb ( Rox_Uchar * buffer, const Rox_Image_RGBA source );

//! @} 

#endif // __OPENROX_ROXRGBA_TO_RGB__
