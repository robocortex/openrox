//==============================================================================
//
//    OPENROX   : File points3d_sphere.h
//
//    Contents  : API of points3d_sphere module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS3D_SPHERE__
#define __OPENROX_POINTS3D_SPHERE__

#include <generated/dynvec_point3d_double.h>

//! \addtogroup Point3D
//! @{

//! Create a list of point which discretize the upper half of a sphere such that they are equally distributed spatially
//! \param  [out] points3d       The list of points allocated and set
//! \param  [in]  nb_subdivs     How many times the original isocahedron is subdivided recursively
//! \param  [in]  maxangle       Maximum angle (in degrees) of the point from the z axis for a point to be valid.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_new_from_sphere(
   Rox_DynVec_Point3D_Double * dynvec_points3d, 
   Rox_Uint nb_subdivs, 
   Rox_Double maxangle);

//! @}

#endif
