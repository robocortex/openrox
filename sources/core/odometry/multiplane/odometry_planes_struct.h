//============================================================================
//
//    OPENROX   : File odometry_planes_struct.h
//
//    Contents  : API of odometry_planes module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ODOMETRY_PLANES_STRUCT__
#define __OPENROX_ODOMETRY_PLANES_STRUCT__

#include <generated/array2d_double.h>
#include <generated/objset_patchplane_pyramid.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

#include <core/model/model_multi_plane.h>
#include <core/odometry/plane/odometry_plane.h>
#include <core/predict/plane_search.h>

//! \ingroup Odometry
//! \defgroup Odometry_Planes Odometry Planes

//! \addtogroup Odometry_Planes
//! @{

//! Multiple plane odometry structure
struct Rox_Odometry_Planes_Struct
{
   //! J'*J buffer
   Rox_Matrix localJtJ;

   //! J'*f buffer
   Rox_Matrix localJtf;

   //! Estimated pose
   Rox_MatSE3 pose;

   //! Homography matrix
   Rox_MatSL3 homography;

   //! Intrinsics of the live camera
   Rox_MatUT3 calib_camera;

   //! Intrinsics of the template camera depending on the level of the puramid
   Rox_Array2D_Double calib_zoom;

   //! List of patches for all planes
   Rox_ObjSet_PatchPlane_Pyramid patches;

   //! Common minimal level of pyramids
   Rox_Uint min_level;

   //! Estimated score ( mean zncc of all planes mapped to [0, 1] )
   Rox_Double score;
   
   //! Score threshold
   Rox_Double score_threshold;

   //! The prediction radius
   Rox_Uint prediction_radius;
      
   //! The maximum number of iterations
   Rox_Uint max_iterations;

   //! The plane predicter
   Rox_Plane_Search * predicter;
};

//! @}

#endif
