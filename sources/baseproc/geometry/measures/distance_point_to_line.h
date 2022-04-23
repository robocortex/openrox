//==============================================================================
//
//    OPENROX   : File distance_point_to_line.h
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

#ifndef __OPENROX_DISTANCE_POINT_TO_LINE__
#define __OPENROX_DISTANCE_POINT_TO_LINE__

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point2d_struct.h>

//! Compute distance between a line and a point in a 2D plane
//! \param  [out] distance    The distance
//! \param  [in]  point       The 2D point
//! \param  [in]  line        The 2D line
//! \return An error code
ROX_API Rox_ErrorCode rox_distance_point2d_to_line2d (
   Rox_Float * distance, 
   const Rox_Point2D_Float point, 
   const Rox_Line2D_Homogeneous
);

//! Compute distance between an epipolar line (line = F * point_cur) and a point point_ref
//! \param  [out] distance    The distance
//! \param  [in]  point_ref   The 2D point
//! \param  [in]  point_cur   The 2D line
//! \param  [in]  fundamental The fundamental matrix F
//! \return the number of set bits.
ROX_API Rox_ErrorCode rox_distance_point2d_to_epipolar_line (
   Rox_Float * distance, 
   const Rox_Point2D_Float point_ref, 
   const Rox_Point2D_Float point_cur, 
   const Rox_Matrix fundamental
);

#endif //__OPENROX_DISTANCE_POINT_TO_LINE__

