//==============================================================================
//
//    OPENROX   : File intersection_line_ellipse.h
//
//    Contents  : 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERSECTION_LINE_ELLIPSE__
#define __OPENROX_INTERSECTION_LINE_ELLIPSE__

#include <generated/array2d_double.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/point/point2d.h>

//! Compute the intersection point between an ellipse and the line
//! passing through a given point and the center of the ellipse
//! \param  [out]  point_intersection   The distance
//! \param  [in ]  ellipse              The 2D ellipse
//! \param  [in ]  point                The 2D point
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse2d_intersection_line_center_point (
   Rox_Point2D_Double point_intersection, 
   Rox_Ellipse2D ellipse, 
   Rox_Point2D_Double point
);

#endif //__OPENROX_INTERSECTION_LINE_ELLIPSE__

