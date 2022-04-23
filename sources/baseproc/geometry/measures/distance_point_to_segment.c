//==============================================================================
//
//    OPENROX   : File distance_point_to_segment.h
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

#include "distance_point_to_segment.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_distance_point2d_to_segment2d (
   Rox_Float * distance,
   const Rox_Point2D_Float point,
   const Rox_Segment2D segment)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *distance = (Rox_Float) rox_distance_point2d_to_segment2d_coordinates ( point->u, point->v, segment->points[0].u, segment->points[0].v, segment->points[1].u, segment->points[1].v );

   return error;
}

// Distance of a point 0 with coordinates (x0,y0) to a segment [1,2] with coordinates (x1,y1) and (x2,y2)
Rox_Double rox_distance_point2d_to_segment2d_coordinates (
   Rox_Double x0,
   Rox_Double y0,
   Rox_Double x1,
   Rox_Double y1,
   Rox_Double x2,
   Rox_Double y2
)
{
   Rox_Double val, denom;

   val = fabs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1));
   denom = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

   return (val/denom);
}
