//============================================================================
//
//    OPENROX   : File draw_ellipse.h
//
//    Contents  : API of draw_ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_DRAW_ELLIPSE__
#define __OPENROX_DRAW_ELLIPSE__

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw the 2D ellipse on the rgba image
//! \param  [out] image_rgba        The rgba image to draw the ellipse
//! \param  [in]  ellipse2d         The 2d ellipse
//! \param  [in]  color             The color of the drawn ellipse
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_ellipse2d(Rox_Image_RGBA image_rgba, Rox_Ellipse2D ellipse2d, Rox_Uint color);

//! Compute and draw the projection of the 3D ellipse on the rgba image
//! \param  [out] image_rgba        The rgba image to draw the ellipse
//! \param  [in]  ellipse3d         The 2d ellipse
//! \param  [in]  calib             The intrinsics parameters
//! \param  [in]  pose              The current point of view
//! \param  [in]  color             The color of the drawn ellipse
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_ellipse3d(Rox_Image_RGBA image_rgba, Rox_Array2D_Double calib, Rox_Array2D_Double pose, Rox_Ellipse3D ellipse3d, Rox_Uint color);

//! @} 

#endif
