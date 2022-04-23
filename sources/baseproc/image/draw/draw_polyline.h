//==============================================================================
//
//    OPENROX   : File draw_polyline.h
//
//    Contents  : API of draw_polyline module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_POLYLINE__
#define __OPENROX_DRAW_POLYLINE__

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

//! Draw a polyline on the rgba image
//! \param [out]  output   The rgba image to draw the polyline
//! \param [in]   pts      The 2D points of the polyline to draw
//! \param [in]   nbpts    The number of polyline points
//! \param [in]   color    The color of the drawn polyline
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_polyline (
   Rox_Image_RGBA output, 
   const Rox_Point2D_Double pts, 
   const Rox_Uint nbpts, 
   const Rox_Uint color
);

//! Compute and draw the projection of the 3D polyline on the rgba image
//! \param  [out]  output   The rgba image to draw the polyline
//! \param  [in ]  calib    The intrinsics parameters
//! \param  [in ]  pose     The current point of view
//! \param  [in ]  pts      The 3D points of the polyline
//! \param  [in ]  nbpts    The number of polyline points
//! \param  [in ]  color    The color of the drawn polyline
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_3d_polyline(
   Rox_Image_RGBA output, 
   const Rox_MatUT3 calib, 
   const Rox_MatSE3 pose, 
   const Rox_Point3D_Double pts, 
   const Rox_Sint nbpts, 
   const Rox_Uint color
);

//! @}

#endif // __OPENROX_DRAW_POLYLINE__
