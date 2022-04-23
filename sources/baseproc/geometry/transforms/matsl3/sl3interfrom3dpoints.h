//==============================================================================
//
//    OPENROX   : File sl3interfrom3dpoints.h
//
//    Contents  : API of sl3interfrom3dpoints module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SL3_INTER_FROM_3D_POINTS__
#define __OPENROX_SL3_INTER_FROM_3D_POINTS__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/plane/plane_struct.h>

//! \ingroup MatSL3
//! \addtogroup sl3from4points
//! @{

//! Retrieve by decomposition the pose and homography which relates the four coplanar 3D vertices to a template of given size
//! \param  [out]  plane          The plane parameters which pass through the 4 points (in original space)
//! \param  [out]  homography     The homography between the [0 0 1 0] plane and the template image
//! \param  [out]  pose           The pose which transforms the four points' plane to the [0 0 1 0] plane
//! \param  [in ]  vertices       The 4 3D points in world frame
//! \param  [in ]  width          The template width
//! \param  [in ]  height         The template height
//! \return An error code
ROX_API Rox_ErrorCode rox_sl3_from_3d_points_double ( 
   Rox_Plane3D_Double plane, 
   Rox_MatSL3 homography, 
   Rox_MatSE3 pose, 
   const Rox_Point3D_Double vertices, 
   const Rox_Sint width, 
   const Rox_Sint height
);

//! @}

#endif
