//==============================================================================
//
//    OPENROX   : File image_rgba_draw.h
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

#ifndef __OPENROX_IMAGE_DISPLAY_DRAW__
#define __OPENROX_IMAGE_DISPLAY_DRAW__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_rect_sint.h>
#include <generated/dynvec_sint.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>

#include <baseproc/image/image_rgba.h>

//! \ingroup Vision
//! \addtogroup Image_Display
//! @{


//! Compute and draw the projection of the 3D polygon on the color image
//! \param  [out]  image          The rgba image to draw the polygon
//! \param  [in ]  calib          The intrinsics parameters
//! \param  [in ]  pose           The current point of view
//! \param  [in ]  pts            The 3D points of the polygon
//! \param  [in ]  nbpts          The number of polygon points
//! \param  [in ]  color          The color of the drawn polygon
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_projection_3d_polygon (
   Rox_Image_RGBA image, 
   const Rox_Matrix calib, 
   const Rox_MatSE3 pose, 
   const Rox_Point3D_Double pts, 
   const Rox_Uint nbpts, 
   const Rox_Uint color
);

//! Draw a polygon on the color image
//! \param  [out]  image          The image to draw the polygon
//! \param  [in ]  pts            The 2D points of the polygon to draw
//! \param  [in ]  nbpts          The number of polygon points
//! \param  [in ]  color          The color (rgba) of the drawn polygon
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_polygon (
   Rox_Image_RGBA image, 
   const Rox_Point2D_Double pts, 
   const Rox_Uint nbpts, 
   const Rox_Uint color
);

//! Draw 2D points on the color image
//! \param  [out]  image          The rgb image to draw the points
//! \param  [in ]  pts            The 2D points to draw
//! \param  [in ]  nbpts          The number of points to draw
//! \param  [in ]  color          The color (rgba) of the drawn line
//! \return An error code
//! \todo   To be tested
// ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_points (
//    Rox_Image_RGBA image, 
//    const Rox_Point2D_Double pts, 
//    const Rox_Uint nbpts, 
//    const Rox_Uint color
// );

ROX_API Rox_ErrorCode rox_image_rgba_draw_dynvec_point2d_double (
   Rox_Image_RGBA image, 
   const Rox_DynVec_Point2D_Double pts, 
   const Rox_Uint color
);

//! Draw 2D points on the color image
//! \param  [out]  image          The rgb image to draw the points
//! \param  [in ]  pts            The 2D points to draw with float coordinates
//! \param  [in ]  nbpts          The number of points to draw
//! \param  [in ]  color          The color (rgba) of the drawn line
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_2d_points_float (
   Rox_Image_RGBA image, 
   const Rox_Point2D_Float pts, 
   const Rox_Uint nbpts, 
   const Rox_Uint color
);

//! Draw a rectangle on the color image
//! \param  [out]  image          The image to draw the rectangle
//! \param  [in ]  rectangle      The rectangle to draw
//! \param  [in ]  color          The color of the drawn line (RGBA value)
//! \return An error code
//! \todo   To be tested
// ROX_API Rox_ErrorCode rox_image_rgba_draw_rectangle (
//    Rox_Image_RGBA image, 
//    const Rox_Rect_Sint rectangle, 
//    const Rox_Uint color
// );

//! Draw a list of rectangles on the color image
//! \param  [out]  image            The image to draw the rectangle
//! \param  [in ]  dynvec_rectangle The rectangle list to draw
//! \param  [in ]  color            The color of the drawn line (RGBA value)
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_dynvec_rectangle (
   Rox_Image_RGBA image, 
   const Rox_DynVec_Rect_Sint dynvec_rectangle, 
   const Rox_Uint color
);

//! Draw a warped rectangle on the color image
//! \param  [out]  image          The image to draw the rectangle
//! \param  [in ]  homography     The homography to warp the rectangle
//! \param  [in ]  rectangle      The rectangle to draw
//! \param  [in ]  color          The color of the drawn rectangles (RGBA value)
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_warp_rectangle (
   Rox_Image_RGBA image, 
   const Rox_MatSL3 homography, 
   const Rox_Rect_Sint rectangle, 
   const Rox_Uint color
);

//! Draw the projection of a 3D frane on the color image
//! The x-axis is blue, the y-axis is green and the z-axis is red
//! \param  [out]  image          The image to draw the model
//! \param  [in ]  calib          The intrinsics parameters
//! \param  [in ]  pose           The current point of view
//! \param  [in ]  size           The size of the frame axes in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_projection_frame ( 
   Rox_Image_RGBA image, 
   const Rox_Matrix calib, 
   const Rox_MatSE3 pose,
   const Rox_Double size
);

//! Create a new image_rgba from two grey images, draw points, and lines between matches
//! \param  [out]  image           The object to create and fill
//! \param  [in ]  image1          The first image
//! \param  [in ]  image2          The second image
//! \param  [in ]  pts1            The points from image1
//! \param  [in ]  pts2            The points from image2
//! \param  [in ]  matches         The matches indices in pts2 for each point in pts1, must contain -1 if no match
//! \param  [in ]  matched_color   The color in which matched points are drawed
//! \param  [in ]  unmatched_color The color in which the points that have no match are drawed
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_matches_new( 
   Rox_Image_RGBA         *image,
   const Rox_Image                  image1,
   const Rox_Image                  image2,
   const Rox_DynVec_Point2D_Double  pts1,
   const Rox_DynVec_Point2D_Double  pts2,
   const Rox_DynVec_Sint            matches,
   const Rox_Uint                   matched_color,
   const Rox_Uint                   unmatched_color 
);

//! @}

#endif // __OPENROX_IMAGE_DISPLAY_DRAW__
