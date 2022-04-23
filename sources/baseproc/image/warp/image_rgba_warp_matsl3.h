//==============================================================================
//
//    OPENROX   : File image_rgba_warp_matsl3.h
//
//    Contents  : API of image display warping module with matrix in SL3
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_RGBA_WARP_MATSL3__
#define __OPENROX_IMAGE_RGBA_WARP_MATSL3__

#include <baseproc/image/image_rgba.h>
#include <baseproc/maths/linalg/matsl3.h>

//! Warp the image with a MatSL3 homography
//! \param  [out]  image_warped   The warped image
//! \param  [in ]  image_source   The source image
//! \param  [in ]  homography		 The homography matrix in SL3
//! \todo   To be tested
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_warp_matsl3 (
   Rox_Image_RGBA image_warped, 
   Rox_Image_RGBA image_source, 
   Rox_MatSL3 homography
);

#endif // __OPENROX_IMAGE_DISPLAY_WARP_MATSL3__
