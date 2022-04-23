//==============================================================================
//
//    OPENROX   : File line_from_planes.h
//
//    Contents  : API of line_from_planes module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_FROM_PLANES__
#define __OPENROX_LINE_FROM_PLANES__

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/plane/plane_struct.h>
#include <system/errors/errors.h>

//! \addtogroup Line
//! @{

//! Build a parametric 3D line from two planes
//! The planes stored in the 3D line mat be not the original ones
//! The first plane will be passing through the origin
//! \param  [out]  line3D_planes     The computed line
//! \param  [in ]  plane3D_1         The first plane
//! \param  [in ]  plane3D_2         The second plane
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_from_2_planes (
   Rox_Line3D_Planes line3D_planes, 
   Rox_Plane3D_Double plane3D_1, 
   Rox_Plane3D_Double plane3D_2
);

//! @}

#endif
