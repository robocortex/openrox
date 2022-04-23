//==============================================================================
//
//    OPENROX   : File odometry_singleplane_light_affine.h
//
//    Contents  : API of odometry_singleplane_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_AFFINE__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_AFFINE__

#include "odometry_singleplane.h"
#include "odometry_singleplane_params.h"

#include <core/patch/patchplane_pyramid.h>
#include <core/odometry/plane/odometry_plane.h>
#include <core/predict/plane_search.h>
#include <core/model/model_single_plane.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Single_Plane_Light_Affine
//! \brief Structure and functions of the model 2D based odometry using an affine light model
//! @{

//! Define the pointer of the Rox_Odometry_Single_Plane_Light_Affine_Struct
typedef struct Rox_Odometry_Single_Plane_Light_Affine_Struct * Rox_Odometry_Single_Plane_Light_Affine;

//! Create the odometry object for localization with repset
//! \param  [out]  odometry         The odometry parameters
//! \param  [in ]  params           The odometry parameters
//! \param  [in ]  model            The 2d model (image + dimensions)
//! \return The pointer to the odometry object
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_light_affine_new (
   Rox_Odometry_Single_Plane_Light_Affine * odometry, 
   const Rox_Odometry_Single_Plane_Params params, 
   const Rox_Model_Single_Plane model
);

//! Delete the odometry object
//! \param  [in ]  odometry         The odometry object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_light_affine_del (
   Rox_Odometry_Single_Plane_Light_Affine * odometry
);

//! Make the odometry
//! \param  [out]  odometry       The odometry object
//! \param  [in ]  camera         Contains the current image and the intrinsic parameters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_light_affine_make (
   Rox_Odometry_Single_Plane_Light_Affine odometry, 
   const  Rox_Camera camera
);

//! Set the template mask
//! \param  [in ]  odometry       The odometry object
//! \param  [in ]  mask           The template mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_light_affine_set_mask (
   Rox_Odometry_Single_Plane_Light_Affine odometry, 
   const Rox_Imask mask
);

//! @}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_LIGHT_AFFINE__
