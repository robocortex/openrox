//==============================================================================
//
//    OPENROX   : File odometry_singleplane_light_affine_struct.h
//
//    Contents  : Strcuture of odometry_singleplane_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_AFFINE_STRUCT__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_AFFINE_STRUCT__

#include "odometry_singleplane_struct.h"
#include "odometry_singleplane.h"
#include "odometry_singleplane_params.h"

#include <core/patch/patchplane_pyramid.h>
#include <core/odometry/plane/odometry_plane_struct.h>
#include <core/predict/plane_search.h>
#include <core/model/model_single_plane.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Single_Plane_Light_Affine
//! \brief Structure and functions of the model 2D based odometry using an affine light model
//! @{

//! The Rox_Odometry_Single_Plane_Light_Affine_Struct object
struct Rox_Odometry_Single_Plane_Light_Affine_Struct
{
   //! The generic odometry structure
   Rox_Odometry_Single_Plane_Struct parent;

   //! The patch pyramid
   Rox_PatchPlane_Pyramid pyramid;

   //! The plane tracker
   Rox_Odometry_Plane tracker;

   //! The plane predicter
   Rox_Plane_Search predicter;
};

//! @}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_AFFINE__
