//==============================================================================
//
//    OPENROX   : File depthfromdisparity.h
//
//    Contents  : API of depthfromdisparity module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DEPTH_FROM_DISPARITY__
#define __OPENROX_DEPTH_FROM_DISPARITY__

#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>

//! \ingroup Vision
//! \defgroup Stereo_Vision Stereo Vision

//! \ingroup StereoVision
//! \addtogroup Disparity
//! @{

//! Compute depth for all pixels of an image pair
//! \param  [out]  depth            the result depth map
//! \param  [out]  output_mask      the result depth map validity mask
//! \param  [in ]  disparity        
//! \param  [in ]  fu_left          the left focal in pixel
//! \param  [in ]  cu_left          the left image center in pixel
//! \param  [in ]  fu_right         the right focal in pixel
//! \param  [in ]  cu_right         the right image center in pixel
//! \param  [in ]  baseline         the distance in meters between the camera
//! \return An error code
ROX_API Rox_ErrorCode rox_depth_from_disparity (
   Rox_Array2D_Float depth, 
   Rox_Array2D_Uint output_mask, 
   const Rox_Array2D_Float disparity, 
   const Rox_Double fu_left, 
   const Rox_Double cu_left, 
   const Rox_Double fu_right, 
   const Rox_Double cu_right, 
   const Rox_Double baseline
);

//! @} 

#endif // __OPENROX_DEPTHFROMDISPARITY__
