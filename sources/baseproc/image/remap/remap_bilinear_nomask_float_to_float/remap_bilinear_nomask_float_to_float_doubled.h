//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_float_to_float_doubled.h
//
//    Contents  : API of remapdouble module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_REMAP_DOUBLED__
#define __OPENROX_REMAP_DOUBLED__

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Resize an image to twice its original dimensions (i.e. the image is doubled).
//! \param  [out]  dest           The resized image (created with double the width and height of source image)
//! \param  [in ]  source         The original image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode remap_bilinear_nomask_float_to_float_doubled ( 
   Rox_Array2D_Float dest, 
   const Rox_Array2D_Float source
);

//! @}

#endif
