//============================================================================
//
//    OPENROX   : File remap_box_halved.h
//
//    Contents  : API of remap_box_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REMAP_BOX_HALVED__
#define __OPENROX_REMAP_BOX_HALVED__

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Resize an image to half its original dimensions using box mean lookup.
//! \param  [out]  dest           The resized image (created with half the width and height of source image)
//! \param  [in ]  source         The original image
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_box_nomask_uchar_to_uchar_halved (
   Rox_Image dest, 
   const Rox_Image source
);

//! Resize an image to half its original dimensions using box mean lookup.
//! \param  [out]  dest           The resized image (created with half the width and height of source image)
//! \param  [in ]  source         The original image
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_box_nomask_float_to_float_halved (
   Rox_Array2D_Float dest, 
   const Rox_Array2D_Float source
);

//! @} 

#endif
