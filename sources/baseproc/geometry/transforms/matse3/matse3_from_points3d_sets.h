//==============================================================================
//
//    OPENROX   : File matse3_from_points3d_sets.h
//
//    Contents  : API of matse3 from point3D sets module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATSE3_FROM_POINTS3D_SETS__
#define __OPENROX_MATSE3_FROM_POINTS3D_SETS__

#include <generated/objset_array2d_double.h>
#include <generated/dynvec_point3d_double.h>

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matse3.h>


//! \ingroup MatSE3
//! \addtogroup se3_from_points_sets
//! @{

//! Given two sets of n ( n >= 3) 3D points, expressed in the reference frame of the first set
//! estimate both a reference frame for the second set and the pose, bTa = [bRa, bta], 
//! between these two such that bTa*A{i} = B{i}
//! \param  [out]  bTa            Computed matse3 pose
//! \param  [in ]  a_points       The first  set of 3D points
//! \param  [in ]  b_points       The second set of 3D points
//! \return An error code
ROX_API Rox_ErrorCode rox_matse3_from_vector_points3d_double ( 
   Rox_MatSE3 bTa,
   const Rox_Point3D_Double a_points,
   const Rox_Point3D_Double b_points, 
   const Rox_Sint n_points
);

ROX_API Rox_ErrorCode rox_matse3_from_dynvec_points3d_double ( 
   Rox_MatSE3 bTa,
   const Rox_DynVec_Point3D_Double a_points,
   const Rox_DynVec_Point3D_Double b_points 
);

//! @}
#endif
