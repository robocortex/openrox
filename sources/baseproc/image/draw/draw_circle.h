//============================================================================
//
//    OPENROX   : File draw_circle.h
//
//    Contents  : API of draw_circle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_DRAW_CIRCLE__
#define __OPENROX_DRAW_CIRCLE__

#include <generated/array2d_uint.h>
#include <baseproc/geometry/point/points_struct.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw the circle defined by its center/radius on the rgba image
//! \param output The rgba image to draw the line
//! \param center_u center point (u coordinate) of the circle to draw
//! \param center_v center point (u coordinate) of the circle to draw
//! \param radius radius of the line to draw
//! \param color The color of the drawn line
//! \return An error code
//! \ŧodo to be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_circle(Rox_Array2D_Uint output, Rox_Sint center_u, Rox_Sint center_v, Rox_Sint radius, Rox_Uint color);

//! @} 

#endif
