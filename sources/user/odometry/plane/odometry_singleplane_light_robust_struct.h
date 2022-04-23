//==============================================================================
//
//    OPENROX   : File odometry_singleplane_light_robust_struct.h
//
//    Contents  : Structure of odometry_singleplane_light_robust module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_ROBUST_STRUCT__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_ROBUST_STRUCT__

#include "odometry_singleplane_params.h"
#include "odometry_singleplane.h"

#include <core/patch/patchplane_robustlight_pyramid.h>
#include <core/odometry/plane/odometry_plane_robustlight.h>
#include <core/odometry/plane/odometry_plane_robustlight_struct.h>
#include <core/predict/plane_search.h>
#include <core/model/model_single_plane.h>

#include <user/odometry/plane/odometry_singleplane_struct.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Single_Plane_Light_Robust
//! \brief  Structure and functions of the model 2D based odometry using a robust light model
//! @{

//! The Rox_Odometry_Single_Plane_Light_Robust_Struct object
struct Rox_Odometry_Single_Plane_Light_Robust_Struct
{
   //! The generic odometry structure
   Rox_Odometry_Single_Plane_Struct parent;

   //! The patch pyramid
   Rox_PatchPlane_RobustLight_Pyramid pyramid;

   //! The plane tracker
   Rox_Odometry_Plane_RobustLight tracker;

   //! The plane predicter
   Rox_Plane_Search predicter;
};

//!@}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_ROBUST_STRUCT__
