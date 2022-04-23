//==============================================================================
//
//    OPENROX   : File draw_polygon.h
//
//    Contents  : API of draw_polygon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_POLYGON__
#define __OPENROX_DRAW_POLYGON__

#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw a polygon on the uchar image
//! \param [out]  output   The rgba image to draw the polygon
//! \param [in]   pts      The 2D points of the polygon to draw
//! \param [in]   nbpts    The number of polygon points
//! \param [in]   color    The color of the drawn polygon
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_polygon (
   Rox_Image_RGBA output, 
   Rox_Point2D_Double pts, 
   Rox_Uint nbpts, 
   Rox_Uint color);

//! Compute and draw the projection of the 3D polygon on the uchar image
//! \param  [out]  output         The rgba image to draw the polygon
//! \param  [in ]  calib          The intrinsics parameters
//! \param  [in ]  pose           The current point of view
//! \param  [in ]  pts            The 3D points of the polygon
//! \param  [in ]  nbpts          The number of polygon points
//! \param  [in ]  color          The color of the drawn polygon
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_projection_3d_polygon (
   Rox_Image_RGBA output, 
   Rox_MatUT3 calib, 
   Rox_MatSE3 pose, 
   Rox_Point3D_Double pts, 
   Rox_Uint nbpts, 
   Rox_Uint color
);

//! @}

#endif // __OPENROX_DRAW_POLYGON__
