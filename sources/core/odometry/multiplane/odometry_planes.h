//============================================================================
//
//    OPENROX   : File odometry_planes.h
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

#ifndef __OPENROX_ODOMETRY_PLANES__
#define __OPENROX_ODOMETRY_PLANES__

#include <core/odometry/multiplane/odometry_planes_struct.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>
#include <generated/array2d_double.h>
#include <core/model/model_multi_plane.h>
#include <core/odometry/plane/odometry_plane.h>
#include <generated/objset_patchplane_pyramid.h>


//! \ingroup Odometry
//! \defgroup Odometry_Planes Odometry Planes

//! \addtogroup Odometry_Planes
//! @{

//! Pointer to multiple plane odometry structure
typedef struct Rox_Odometry_Planes_Struct * Rox_Odometry_Planes;

//! Create a new odometry object for multiplane
//! \param  [out]  odometry_planes        a pointer to the newly created object
//! \param  [in ]  model                  the 3d model to use ( copied inside the object so any further modification on the model won't have any effect on this odometry object )
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_new ( 
   Rox_Odometry_Planes * odometry_planes, 
   const Rox_Model_Multi_Plane model 
);

//! Delete an odometry object for multiplane
//! \param  [out]  odometry_planes            The pointer to the object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_del ( 
   Rox_Odometry_Planes * odometry_planes 
);

//! Perform odometry using an odometry object for multiplane
//! \param  [in ]  odometry_planes  The odometry object to use
//! \param  [in ]  model            The model to use
//! \param  [in ]  image            The image in which track the object 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_make (
   Rox_Odometry_Planes odometry_planes, 
   const Rox_Model_Multi_Plane    model, 
   const Rox_Array2D_Float   image
);

//! Get pose  estimated by last make
//! \param  [out]  pose           The estimated pose
//! \param  [in ]  odometry_planes            The odometry object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_get_pose ( 
   Rox_Array2D_Double pose, 
   const Rox_Odometry_Planes odometry_planes 
);

//! Get score estimated by last make
//! \param  [out]  score          The estimated score
//! \param  [in ]  odometry_planes            The odometry object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_get_score ( 
   Rox_Double * score, 
   const Rox_Odometry_Planes odometry_planes 
);

//! Get result
//! \param  [out]  is_tracked                The flag to knpw if is tracked
//! \param  [out]  score                     The estimated score
//! \param  [out]  pose                      The estimated pose
//! \param  [in ]  odometry_planes           The odometry object to use
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_get_result (
   Rox_Sint   * is_tracked,
   Rox_Double * score, 
   Rox_MatSE3 pose, 
   const Rox_Odometry_Planes odometry_planes
);


//! Set plane pose + calibration
//! \param  [in ]  odometry_planes           The tracking object
//! \param  [in ]  pose                      The pose matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_set_pose ( 
   Rox_Odometry_Planes odometry_planes, 
   const Rox_MatSE3 pose 
);

//! Set the camera calibration
//! \param  [in ]  odometry_planes                     The tracking object
//! \param  [in ]  calibration_camera      The calibration matrix for camera
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_planes_set_camera_calibration ( 
   Rox_Odometry_Planes odometry_planes, 
   const Rox_MatUT3 calibration_camera 
);

//! @}

#endif
