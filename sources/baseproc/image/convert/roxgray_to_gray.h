//==============================================================================
//
//    OPENROX   : File roxgray_to_gray.h
//
//    Contents  : API of roxgray_to_gray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROXGRAY_TO_GRAY__
#define __OPENROX_IMAGE_ROXGRAY_TO_GRAY__

#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//!   @{

//! Convert a rox image to grayscale buffer
//! \param [out] dest The grayscale buffer
//! \param [in] input The image to copy
//! \return An error code
//! \todo an error code
ROX_API Rox_ErrorCode rox_roxgray_to_gray ( Rox_Uchar * dest, const Rox_Array2D_Uchar input );

//! @} 

#endif // __OPENROX_IMAGE_ROXGRAY_TO_GRAY__
