//==============================================================================
//
//    OPENROX   : File draw_warp_polygon.h
//
//    Contents  : API of draw_warp_polygon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_WARP_POLYGON__
#define __OPENROX_DRAW_WARP_POLYGON__

#include <generated/array2d_double.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw a warped polygon on the uchar image
//! \param output The rgba image to draw the polygon
//! \param H      The image-to-image homography to warp the 2d polygon
//! \param pts    The 2D points of the polygon to draw
//! \param nbpts  The number of polygon points
//! \param color  The color of the drawn polygon (rgba)
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_warp_polygon(
   Rox_Image_RGBA output, 
   Rox_Array2D_Double H, 
   Rox_Point2D_Double  pts, Rox_Uint nbpts, Rox_Uint color);

//! Draw a warped-projected polygon on the uchar image
//! \param output The rgba image to draw the polygon
//! \param H      The model-to-image homography to warp the 3d polygon
//! \param pts    The 3D points of the polygon to draw
//! \param nbpts  The number of polygon points
//! \param color  The color of the drawn polygon (rgba)
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_warp_projection_polygon_3d(
   Rox_Image_RGBA obj, 
   Rox_Array2D_Double H, 
   Rox_Point3D_Double  pts, 
   Rox_Uint nbpts, Rox_Uint color);

//! @}

#endif // __OPENROX_DRAW_WARP_POLYGON__
