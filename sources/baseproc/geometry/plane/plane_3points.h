//==============================================================================
//
//    OPENROX   : File plane_from_3_point3d.h
//
//    Contents  : API of plane_3points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLANE3D_FROM_3_POINT3D__
#define __OPENROX_PLANE3D_FROM_3_POINT3D__

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/plane/plane_struct.h>

//! \addtogroup Plane
//! @{

//! Given 3 points, compute the plane which pass through these points (At least one of the possible points).
//! The normal orientation is sensitive to the order of the points.
//! \param  [out]  plane          The result plane
//! \param  [in ]  pt1            The first points in 3D coordinates
//! \param  [in ]  pt2            The second points in 3D coordinates
//! \param  [in ]  pt3            The third points in 3D coordinates
//! \return An error code
ROX_API Rox_ErrorCode rox_plane3d_from_3_point3d ( 
   Rox_Plane3D_Double plane, 
   const Rox_Point3D_Double pt1, 
   const Rox_Point3D_Double pt2, 
   const Rox_Point3D_Double pt3
);

//! Given 3 points, compute the plane which pass through these points (At least one of the possible points).
//! The normal orientation is sensitive to the order of the points.
//! \param  [out]  plane          The result plane
//! \param  [in ]  pt1            The first points in 3D coordinates
//! \param  [in ]  pt2            The second points in 3D coordinates
//! \param  [in ]  pt3            The third points in 3D coordinates
//! \return An error code
ROX_API Rox_ErrorCode rox_plane3d_from_3_point3d_float ( 
   Rox_Plane3D_Double plane, 
   const Rox_Point3D_Float pt1, 
   const Rox_Point3D_Float pt2, 
   const Rox_Point3D_Float pt3
);

//! @} 

#endif