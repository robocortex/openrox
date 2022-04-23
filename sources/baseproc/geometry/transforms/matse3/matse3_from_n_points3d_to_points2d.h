//==============================================================================
//
//    OPENROX   : File matse3_from_n_points3d_to_points2d.h
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

#ifndef __OPENROX_MATSE3_FROM_N_POINTS3D_TO_POINTS2D__
#define __OPENROX_MATSE3_FROM_N_POINTS3D_TO_POINTS2D__

#include <generated/objset_array2d_double.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matse3.h>

//! \ingroup MatSE3
//! \addtogroup se3_from_n_points
//! @{

//! Generate a pose T = [R, t] given nbp >= 3 points in 3D and the corresponding 2D points
//! such that min_{R, t, z} sum^n_{k=1} {|| R * m_k + t - z_k * q_k ||^2} 
//! \param  [out]  T              Computed matse3 pose 
//! \param  [out]  z              Computed depths z 
//! \param  [in ]  q              The 2D points in the image frame
//! \param  [in ]  m              The 3D points in the world frame
//! \param  [in ]  n              The number of points 
//! \return An error code
ROX_API Rox_ErrorCode rox_matse3_from_n_points3d_to_points2d_double(Rox_ObjSet_Array2D_Double T, Rox_ObjSet_Array2D_Double z, const Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Sint n);

ROX_API Rox_ErrorCode rox_cost_function(Rox_Double * cost, Rox_MatSE3 T, Rox_Array2D_Double z, Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Uint nbp);

//! @}

#endif // __OPENROX_MATSE3_FROM_N_POINTS3D_TO_POINTS2D__
