//==============================================================================
//
//    OPENROX   : File rox_image_rgba_draw_projection_model_multi_plane.h
//
//    Contents  : API of image_rgba_draw module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_DISPLAY_DRAW_PROJECTION_MODEL_MULTI_PLANE__
#define __OPENROX_IMAGE_DISPLAY_DRAW_PROJECTION_MODEL_MULTI_PLANE__

#include <baseproc/image/draw/image_rgba_draw.h>
#include <core/model/model_multi_plane.h>

//! Draw the projection of a multi plane model on the color image
//! \param  [out]  image          The image to draw the model
//! \param  [in ]  calib          The intrinsics parameters
//! \param  [in ]  pose           The current point of view
//! \param  [in ]  model          The single plane model to be projected in the image
//! \param  [in ]  color          The color of the drawn line (RGBA value) 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_projection_model_multi_plane (
   Rox_Image_RGBA image, 
   const Rox_Matrix calib, 
   const Rox_MatSE3 pose, 
   const Rox_Model_Multi_Plane model, 
   const Rox_Uint color
);

#endif // __OPENROX_IMAGE_DISPLAY_DRAW_PROJECTION_MODEL_MULTI_PLANE__
