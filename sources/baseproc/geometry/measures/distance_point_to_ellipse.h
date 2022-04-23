//==============================================================================
//
//    OPENROX   : File distance_point_to_ellipse.h
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

#ifndef __OPENROX_DISTANCE_POINT_TO_ELLIPSE__
#define __OPENROX_DISTANCE_POINT_TO_ELLIPSE__

#include <generated/array2d_double.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/point/point2d.h>

//! Compute distance between a line and a point in a 2D plane
//! \param  [out]  distance    The distance
//! \param  [in ]  point       The 2D point
//! \param  [in ]  line        The 2D ellipse
//! \return An error code
ROX_API Rox_ErrorCode rox_distance_point2d_to_ellipse2d_algebraic (
   Rox_Double * distance, Rox_Point2D_Double point, Rox_Ellipse2D_Parametric ellipse);

//! \return An error code
//! \param  [out]  signed_distance   The signed distance
//! \param  [in ]  ellipse2d         The ellipse 2D
//! \param  [in ]  point2d           The point 2D
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_signed_distance_point2d_to_ellipse2d_algebraic (
   Rox_Double * signed_distance, 
   Rox_Ellipse2D ellipse2d, 
   Rox_Point2D_Double point2d
);

#endif //__OPENROX_DISTANCE_POINT_TO_ELLIPSE__

