//==============================================================================
//
//    OPENROX   : File rectangle_detection.h
//
//    Contents  : API of rectangle detection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RECTANGLE_DETECTION__
#define __OPENROX_RECTANGLE_DETECTION__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 
   
// ====== INCLUDED HEADERS   ===================================================

#include <baseproc/image/image.h>
#include <baseproc/geometry/point/point2d.h>

// ====== EXPORTED MACROS    ===================================================

// ====== INTERNAL MACROS    ===================================================

// ====== EXPORTED DATATYPES ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== EXPORTED FUNCTIONS ===================================================

//! \ingroup  Vision
//! \defgroup Image_Processing_Tools Image Processing Tools
//! \brief Image processing tools structures and methods.

//! \addtogroup Image_Processing_Tools
//! @{

//! Init a white rectangle detector (allocate memory)
//! \param  [in ]  cols     The width of the image in which the rectangle will be detected
//! \param  [in ]  rows     The height of the image in which the rectangle will be detected
//! \param  [in ]  fu       The camera u-axis focal length in pixel
//! \param  [in ]  fv       The camera v-axis focal length in pixel
//! \param  [in ]  cu       The camera u-axis principal point in pixels
//! \param  [in ]  cv       The camera v-axis principal point in pixels
//! \return An error code
ROX_API Rox_ErrorCode rox_rectangle_detector_init (
   Rox_Sint cols, 
   Rox_Sint rows, 
   Rox_Double fu, 
   Rox_Double fv, 
   Rox_Double cu, 
   Rox_Double cv
);

//! Process an image to detect a white rectangle 
//! \param  [out]  is_detected       The flag to know if the rectangle is detected (1) or not (0) 
//! \param  [out]  pts_rect          The coordinates of the 4 points of the rectangle. The first and second point of the rectangle defines the longer side.
//! \param  [in ]  image             The input image in which detect a white rectangle. 
//! \param  [in ]  ratio_rect        The ratio between the sizes of the rectangle.
//! \param  [in ]  detection_method  The method to be used : 0 for low constrast images (slow), 1 for high constrast images (fast).
//! \param  [in ]  mean_min          The minimum value of its pixels color values average authorized for an estimated rectangle. ie The minimum "whiteness" threshold to consider a rectangle valid.
//! \param  [in ]  variance_max      The maximum variance between its pixels color values to consider a rectangle valid.
//! \return An error code
ROX_API Rox_ErrorCode rox_rectangle_detector_process (
   Rox_Sint * is_detected, 
   Rox_Point2D_Double pts_rect, 
   Rox_Image image, 
   Rox_Double ratio_rect, 
   Rox_Sint detection_method, 
   Rox_Sint mean_min, 
   Rox_Sint variance_max
);

//! Terminate the rectangle detection (free allocated memory) 
//! \return An error code
ROX_API Rox_ErrorCode rox_rectangle_detector_terminate();

//! @} 

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __OPENROX_DETECTION_MOTION__