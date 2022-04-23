//==============================================================================
//
//    OPENROX   : File matse3_from_n_points3d_to_planes3d.h
//
//    Contents  : API of matse3 from points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATSE3_FROM_N_POINTS3D_TO_PLANES3D__
#define __OPENROX_MATSE3_FROM_N_POINTS3D_TO_PLANES3D__

#include <generated/objset_array2d_double.h>

#include <baseproc/geometry/plane/plane_struct.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matse3.h>

//! \ingroup MatSE3
//! \addtogroup se3_from_n_points
//! @{

//! Generate a pose T = [R, t] given nbp >= 3 points in 3D and the corresponding 2D points
//! such that min_{R, t, z} sum^n_{k=1} {|| R * m_k + t - z_k * q_k ||^2} 
//! \param  [out]  T              Computed matse3 pose 
//! \param  [in ]  p              The 3D planes in the image frame
//! \param  [in ]  m              The 3D points in the world frame
//! \param  [in ]  n              The number of points 
//! \return An error code
ROX_API Rox_ErrorCode rox_matse3_from_n_points3d_to_planes3d_double(Rox_ObjSet_Array2D_Double T, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Sint n);

ROX_API Rox_ErrorCode rox_cost_function_points3d_to_planes3d(Rox_Double * cost, Rox_MatSE3 T, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Uint nbp);

//! @}

#endif // __OPENROX_MATSE3_FROM_N_POINTS3D_TO_PLANES3D__
