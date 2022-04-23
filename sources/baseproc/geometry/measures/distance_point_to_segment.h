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

#ifndef __OPENROX_DISTANCE_POINT_TO_SEGMENT__
#define __OPENROX_DISTANCE_POINT_TO_SEGMENT__

#include <baseproc/geometry/segment/segment2d_struct.h>
#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>

//! Compute distance between a segment and a point in a 2D plane
//! \param  [out]  distance       The distance
//! \param  [in ]  point          The 2D point
//! \param  [in ]  segment        The 2D segment
//! \return An error code
ROX_API Rox_ErrorCode rox_distance_point2d_to_segment2d ( 
   Rox_Float * distance, 
   const Rox_Point2D_Float point, 
   const Rox_Segment2D segment 
);

//! Compute distance between a segment
//! \param  [out] ptr pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_Double rox_distance_point2d_to_segment2d_coordinates ( 
   Rox_Double x0, 
   Rox_Double y0, 
   Rox_Double x1, 
   Rox_Double y1, 
   Rox_Double x2, 
   Rox_Double y2 
);

#endif //__OPENROX_DISTANCE_POINT_TO_SEGMENT__

