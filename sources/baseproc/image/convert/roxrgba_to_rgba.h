//==============================================================================
//
//    OPENROX   : File roxrgba_to_rgba.h
//
//    Contents  : API of roxrgba_to_rgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROXRGBA_TO_RGBA__
#define __OPENROX_IMAGE_ROXRGBA_TO_RGBA__

#include <generated/array2d_uint.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//!   @{

//! Convert a color image to RGBA buffer
//! \param [out]  buffer         The destination buffer
//! \param [in]   source         The source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxrgba_to_rgba(Rox_Uchar * buffer, const Rox_Array2D_Uint source);

//! @} 

#endif // __OPENROX_ROXRGBA_TO_RGBA__
