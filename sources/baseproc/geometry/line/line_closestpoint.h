//==============================================================================
//
//    OPENROX   : File line_closestpoint.h
//
//    Contents  : API of line_closestpoint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_CLOSESTPOINT__
#define __OPENROX_LINE_CLOSESTPOINT__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/segment/segment3d.h>
#include <baseproc/geometry/line/line3d.h>

#include <system/errors/errors.h>

//! \addtogroup Line
//! @{

//! Compute the 3D point which is the point on the 3D segment which is closest to the emitted ray
//! \param  [out]  point3d        The 3D point on the segment which is closest to the ray
//! \param  [in ]  segment3d      The segment to use as a line
//! \param  [in ]  ray            The ray to intersect with the line
//! \return An error code
ROX_API Rox_ErrorCode rox_segment3d_backproject (
   Rox_Point3D_Double res, 
   const Rox_Segment3D segment, 
   const Rox_Point2D_Double ray
);

//! Compute the 3D point which is the point on the 3D segment which is closest to the emitted ray
//! \param  [out]  point3d        The point on the segment line which is closest to the ray
//! \param  [in ]  segment3d      The segment to use as a line
//! \param  [in ]  ray            The ray to intersect with the line
//! \return An error code
ROX_API Rox_ErrorCode rox_segment3d_float_backproject (
   Rox_Point3D_Float res, 
   const Rox_Segment3D_Float segment3d, 
   const Rox_Point2D_Float ray
);

ROX_API Rox_ErrorCode rox_point3d_closests_from_2_lines3d (
   Rox_Point3D_Double pts_line1,
   Rox_Point3D_Double pts_line2,
   const Rox_Line3D_Parametric line3d1, 
   const Rox_Line3D_Parametric line3d2
);

//! @}

#endif
