//==============================================================================
//
//    OPENROX   : File harris.h
//
//    Contents  : API of harris module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_HARRIS__
#define __OPENROX_HARRIS__

#include <generated/array2d_float.h>
#include <baseproc/image/image.h>

//! \ingroup Detectors
//! \addtogroup Harris
//! @{

//! Harris corner detector
//! \param  [out]  response 		 The output corner response
//! \param  [in ]  source			 The raw image data pointer.
//! \param  [in ]  window_radius	 The radius of the patch to consider for each pixel
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_harris_detector_uchar ( 
   Rox_Array2D_Float response, 
   const Rox_Image source, 
   const Rox_Uint window_radius
);

//! @}

#endif
