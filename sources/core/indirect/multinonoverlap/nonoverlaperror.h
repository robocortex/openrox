//==============================================================================
//
//    OPENROX   : File nonoverlaperror.h
//
//    Contents  : API of nonoverlaperror module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_NONOVERLAP_ERROR__
#define __OPENROX_NONOVERLAP_ERROR__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_double.h>

//! \ingroup Geometry
//! \addtogroup NonOverlap
//! @{

//! Given an Essential and a pose matrix, compute the per point arithmetic error
//! \param vecerrors error vector
//! \param E essential matrix
//! \param pose pose matrix
//! \param ar reference plucker
//! \param br reference plucker
//! \param ac reference plucker
//! \param bc reference plucker
//! \return An error code
ROX_API Rox_ErrorCode rox_nonoverlap_geometric_error(Rox_Array2D_Double vecerrors, Rox_Array2D_Double E, Rox_Array2D_Double pose, Rox_DynVec_Point2D_Double ar, Rox_DynVec_Point3D_Double br, Rox_DynVec_Point2D_Double ac, Rox_DynVec_Point3D_Double bc);

//! @} 

#endif
