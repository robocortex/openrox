//==============================================================================
//
//    OPENROX   : File image_roi.h
//
//    Contents  : API of image roi (region of interest)
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROI__
#define __OPENROX_IMAGE_ROI__

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/geometry/rectangle/rectangle.h>

//! Create a new image from an roi (region of interest)
//! \param [in]   image_ref      The warped image float
//! \param [in]   imask_ref      The imask of the warped image
//! \param [in]   roi            The source image float
//! \param [in]   image          The homography matrix in SL3
//! \param [in]   imask          The homography matrix in SL3
//! \todo To be tested
//! \return An error code
ROX_API Rox_ErrorCode rox_image_new_roi ( 
   Rox_Image * image_ref, 
   Rox_Imask * imask_ref, 
   Rox_Rect_Sint_Struct * roi,
   const Rox_Image image,
   const Rox_Imask imask 
);

//! Copy a region of interest (roi) of an image in another 
//! \param  [out]  dest           The destination image
//! \param  [in ]  roi            The region of interest (roi) 
//! \param  [in ]  source         The source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_new_copy_roi ( Rox_Image * dest, const Rox_Rect_Sint roi, const Rox_Image source );

#endif // __OPENROX_IMAGE_WARP_MATSL3__
