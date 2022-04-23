//==============================================================================
//
//    OPENROX   : File line_clip.h
//
//    Contents  : API of line_clip module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_CLIP__
#define __OPENROX_LINE_CLIP__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/geometry/point/point2d.h>

#include <system/errors/errors.h>

//! \addtogroup Line
//! @{

//! Clip a homogeneous line inside a rectangle
//! \param  [out]  pt1            The computed first point
//! \param  [out]  pt2            The computed second point
//! \param  [in ]  line           The line
//! \param  [in ]  rect           The rectangle
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_clip_inside_rectangle (
   Rox_Point2D_Double pt1, 
   Rox_Point2D_Double pt2, 
   const Rox_Line2D_Homogeneous line, 
   const Rox_Rect_Real rect
);

//! @}

#endif
