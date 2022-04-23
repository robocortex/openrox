//==============================================================================
//
//    OPENROX   : File roxrgba_split.h
//
//    Contents  : API of roxrgba_split module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROXRGBA_SPLIT__
#define __OPENROX_IMAGE_ROXRGBA_SPLIT__

#include <baseproc/image/image.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a rgba rox internal image to the 3 color planes images
//! \param  [out]  image_red      red destination image
//! \param  [out]  image_green    green destination image
//! \param  [out]  image_blue     blue destination image
//! \param  [in ]  image_rgba     source image
//! \return An error code
ROX_API Rox_ErrorCode rox_roxrgba_split ( 
   Rox_Image image_red, 
   Rox_Image image_green, 
   Rox_Image image_blue, 
   const Rox_Image_RGBA image_rgba
);

//! @} 

#endif // __OPENROX_IMAGE_ROXRGBA_SPLIT__
