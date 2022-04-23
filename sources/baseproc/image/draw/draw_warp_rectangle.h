//==============================================================================
//
//    OPENROX   : File draw_warp_rectangle.h
//
//    Contents  : API of draw_warp_rectangle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_WARP_RECTANGLE__
#define __OPENROX_DRAW_WARP_RECTANGLE__

#include <generated/array2d_double.h>
#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw a warped rectangle on the uchar image
//! \param  [out] output         The uchar image to draw the rectangle
//! \param  [in]  H              The homography to warp the polygon
//! \param  [in]  rectangle      The rectangle to draw
//! \param  [in]  rgbvalue       The color of the drawn line (RGBA value)
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_warp_rectangle(Rox_Image_RGBA output, Rox_Array2D_Double H, Rox_Rect_Sint rectangle, Rox_Uint rgbvalue);

//! @}

#endif // __OPENROX_DRAW_WARP_RECTANGLE__
