//==============================================================================
//
//    OPENROX   : File draw_checkercorner_detector_corners.h
//
//    Contents  : API of draw_checkercorner_detector_corners module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//    
//==============================================================================

#ifndef __OPENROX_CHECKERCORNER_DETECTOR_CORNERS__
#define __OPENROX_CHECKERCORNER_DETECTOR_CORNERS__

#include <baseproc/image/image_rgba.h>
#include <core/features/detectors/checkerboard/checkercorner_detect.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw 2D points (a 2x2 cross) on the rgb image.
//! \param  [out]  image_rgba                The rgba image to draw the points
//! \param  [in ]  checkercorner_detector    The checkerboard detector containing the 2D points to draw
//! \param  [in ]  color                     The color (rgba) of the drawn corners
//! \return An error code
ROX_API Rox_ErrorCode rox_checkercorner_detector_draw_corners (
   Rox_Image_RGBA image_rgba, 
   Rox_CheckerCorner_Detector checkercorner_detector, 
   Rox_Uint color
);

//! @}

#endif // __OPENROX_CHECKERCORNER_DETECTOR_CORNERS__
