//============================================================================
//
//    OPENROX   : File remaphalvednn.h
//
//    Contents  : API of remaphalvednn module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_REMAPHALVEDNN__
#define __OPENROX_REMAPHALVEDNN__

#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <baseproc/image/image.h>

//! Resize an image to half its original dimensions using simple nearest neighboor lookup.
//! \param  [out]  dest           The resized image (created with half the width and height of source image)
//! \param  [in ]  source         The original image
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_remap_halved_nn (
   Rox_Image dest, 
   const Rox_Image source
);

//! Resize an image to half its original dimensions using simple nearest neighboor lookup.
//! \param  [out]  dest           The resized image (created with half the width and height of source image)
//! \param  [in ]  source         The original image
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_array2d_float_remap_halved_nn (
   Rox_Array2D_Float dest, const Rox_Array2D_Float source
);

//! Resize an image to half its original dimensions using simple nearest neighboor lookup.
//! \param  [out]  dest           The resized image (created with half the width and height of source image)
//! \param  [in ]  source         The original image
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_array2d_uint_remap_halved_nn (
   Rox_Array2D_Uint dest, const Rox_Array2D_Uint source
);

#endif
