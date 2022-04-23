//==============================================================================
//
//    OPENROX   : File draw_points.h
//
//    Contents  : API of draw_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_POINTS__
#define __OPENROX_DRAW_POINTS__

#include <generated/dynvec_point2d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <core/features/detectors/segment/segmentpoint.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw 2D points (a 2x2 cross) on the rgb image.
//! \param  [out]  output      The rgb image to draw the points
//! \param  [in ]  pts         The 2D points to draw
//! \param  [in ]  nbpts       The number of points to draw
//! \param  [in ]  color       The color (rgba) of the drawn line
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_points (
   Rox_Image_RGBA output, 
   Rox_Point2D_Double pts, 
   Rox_Uint nbpts, 
   Rox_Uint color
);

//! Draw 2D points (a 2x2 cross) on the rgb image.
//! \param  [out]  output      The rgb image to draw the points
//! \param  [in ]  pts         The 2D points to draw
//! \param  [in ]  nbpts       The number of points to draw
//! \param  [in ]  color       The color (rgba) of the drawn line
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_dynvec_point2d_double (
   Rox_Image_RGBA output, 
   Rox_DynVec_Point2D_Double pts, 
   Rox_Uint color
);

//! Draw 2D points (a 2x2 cross) on the rgb image.
//! \param  [out]  output         The rgb image to draw the points
//! \param  [in ]  pts            The 2D points to draw
//! \param  [in ]  nbpts          The number of points to draw
//! \param  [in ]  color          The color (rgba) of the drawn line
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_points_float (
   Rox_Image_RGBA output, 
   Rox_Point2D_Float pts, 
   Rox_Uint nbpts, 
   Rox_Uint color
);

//! Draw 2D points (a 2x2 cross) on the rgb image.
//! \param  [out]  output         The rgb image to draw the points
//! \param  [in ]  pts            The 2D points to draw
//! \param  [in ]  nbpts          The number of points to draw
//! \param  [in ]  color          The color (rgba) of the drawn line
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_points_sint (
   Rox_Image_RGBA output, 
   Rox_Point2D_Sint pts, 
   Rox_Uint nbpts, 
   Rox_Uint color
);

//! Draw 2D points (a 2x2 cross) on the rgb image.
//! \param  [out]  output         The rgb image to draw the points
//! \param  [in ]  pts            The 2D segment points to draw
//! \param  [in ]  nbpts          The number of points to draw
//! \param  [in ]  color          The color (rgba) of the drawn line
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_segment_points (
   Rox_Image_RGBA output, 
   Rox_Segment_Point pts, 
   Rox_Uint nbpts, 
   Rox_Uint color
);

ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_points_array_float (
   Rox_Image_RGBA image_rgba,
   Rox_Float * pts,
   Rox_Uint nbpts,
   Rox_Uint color
);

//! @}

#endif // __OPENROX_DRAW_POINTS__
