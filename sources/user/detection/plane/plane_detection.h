//==============================================================================
//
//    OPENROX   : File plane_detection.h
//
//    Contents  : Plane detection module definition
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLANE_DETECTION__
#define __OPENROX_PLANE_DETECTION__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 
   
//=== INCLUDED HEADERS   =======================================================

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/geometry/point/point2d.h>

//=== EXPORTED MACROS    =======================================================

//=== INTERNAL MACROS    =======================================================

//=== EXPORTED DATATYPES =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== EXPORTED FUNCTIONS =======================================================

//! \addtogroup Image_Processing_Tools
//! @{

//! Detect pixels on the same plane of a rectangle
//! \param [out]   mask1          The mask containing the pixels of the image1 on the same plane of the rectangle  
//! \param [out]   mask2          The mask containing the pixels of the image2 on the same plane of the rectangle  
//! \param [in]    pts_rect_1     The 4 coordinates of the rectangle extracted in image 1
//! \param [in]    pts_rect_2     The 4 coordinates of the rectangle extracted in image 2
//! \param [in]    image1         The input image 1
//! \param [in]    image2         The input image 2
//! \param [in]    threshold      The threshold in grayscale level to decide if a pixel is on the plane
//! \return An error code
ROX_API Rox_ErrorCode rox_plane_detector_from_rectangles (
   Rox_Imask mask1, 
   Rox_Imask mask2, 
   Rox_Point2D_Double pts_rect_1, 
   Rox_Point2D_Double pts_rect_2, 
   Rox_Image image1, 
   Rox_Image image2, 
   Rox_Sint threshold
);

//! @}

#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_PLANE_DETECTION__
