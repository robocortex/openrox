//==============================================================================
//
//    OPENROX   : File draw_line.h
//
//    Contents  : API of draw_line module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DRAW_LINE__
#define __OPENROX_DRAW_LINE__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/image/image_rgba.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw the line defined by the 2 points on the rgba image
//! \param  [out]  image_rgba     The rgba image to draw the line
//! \param  [in ]  point2d_1      First point of the line to draw
//! \param  [in ]  point2d_2      Second point of the line to draw
//! \param  [in ]  color          The color of the drawn line
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_line (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Double point2d_1, 
   const Rox_Point2D_Double point2d_2, 
   const Rox_Uint color
);


//! Draw the line defined by the 2 points on the rgba image
//! \param  [out]  image_rgba     The rgba image to draw the line
//! \param  [in ]  point2d_1      First point of the line to draw
//! \param  [in ]  point2d_2      Second point of the line to draw
//! \param  [in ]  color          The color of the drawn line
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_line_float (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Float point2d_1, 
   const Rox_Point2D_Float point2d_2, 
   const Rox_Uint color
);

//! Draw the line defined by the 2 points on the rgba image
//! \param  [out]  image_rgba     The rgba image to draw the line
//! \param  [in ]  point2d_1      First point of the line to draw
//! \param  [in ]  point2d_2      Second point of the line to draw
//! \param  [in ]  color          The color of the drawn line
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_line_sint (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Sint point2d_1, 
   const Rox_Point2D_Sint point2d_2, 
   const Rox_Uint color
);

//! Draw the line defined by the 2 points on the rgba image.
//! \param  [out]  image_rgba     The rgba image to draw the line
//! \param  [in ]  point2d_1      First point of the line to draw
//! \param  [in ]  point2d_2      Second point of the line to draw
//! \param  [in ]  color          The color of the drawn line
//! \param  [in ]  thickness      The line thickness (a odd number)
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_line_thick(
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Sint point2d_1, 
   const Rox_Point2D_Sint point2d_2, 
   const Rox_Uint color, 
   const Rox_Uint thickness
);

ROX_API Rox_ErrorCode rox_image_rgba_draw_segment2d (
   Rox_Image_RGBA image_rgba, 
   const Rox_Segment2D segment2d, 
   const Rox_Uint color
);

//! @}

#endif
