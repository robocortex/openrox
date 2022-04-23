//==============================================================================
//
//    OPENROX   : File draw_rectangle.h
//
//    Contents  : API of draw_rectangle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_RECTANGLE__
#define __OPENROX_DRAW_RECTANGLE__

#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw a rectangle on the uchar image
//! \param output The rgba image to draw the rectangle
//! \param rectangle The rectangle to draw
//! \param color The color of the drawn line
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_rectangle(Rox_Image_RGBA output, Rox_Rect_Sint rectangle, Rox_Uint color);

//! @}

#endif
