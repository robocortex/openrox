//==============================================================================
//
//    OPENROX   : File pointcloud_similarity.h
//
//    Contents  : API of pointcloud_similarity module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PC_SIMILARITY__
#define __OPENROX_PC_SIMILARITY__

#include <generated/array2d_double.h>
#include <generated/dynvec_point3d_double.h>

//! \ingroup Geometry
//! \addtogroup euclidean
//! @{

//! Compute the similarity transform between 2 point clouds (Minimizing the point to point squared L2 distance for each point pair). Use the algorithm described in  Least squares estimation of transformation parameters between two point patterns (Shinji Umeyama).
//! \param  []  similarity
//! \param  []  ref3D            the n points of the reference point cloud
//! \param  []  cur3D            the n points of the current point cloud
//! \return An error code
ROX_API Rox_ErrorCode rox_pointcloud_similarity(Rox_Array2D_Double similarity, Rox_DynVec_Point3D_Double ref3D, Rox_DynVec_Point3D_Double cur3D);

//! @} 

#endif
