//==============================================================================
//
//    OPENROX   : File odometry_multiplane.h
//
//    Contents  : API of odometry_multiplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_MULTI_PLANE__
#define __OPENROX_ODOMETRY_MULTI_PLANE__

#include <baseproc/maths/linalg/matse3.h>

#include <core/model/model_multi_plane.h>

#include <user/odometry/multiplane/odometry_multiplane_params.h>
#include <user/sensor/camera/camera.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Multi_Plane
//! @{

//! Define the pointer of the Rox_Odometry_Planes_Struct
typedef struct Rox_Odometry_Planes_Struct * Rox_Odometry_Multi_Plane ;


//! Create a new odometry object for multiplane
//! \param  [out]  odometry_multi_plane   The odometry multiplane object
//! \param  [in ]  model_3d               3D model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_new ( 
   Rox_Odometry_Multi_Plane * odometry_multi_plane, 
   const Rox_Odometry_Multi_Plane_Params params,
   const Rox_Model_Multi_Plane model_3d 
);


//! Delete an odometry object for multiplane
//! \param  [out]  odometry_multi_plane    The pointer to the object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_del ( 
   Rox_Odometry_Multi_Plane * odometry_multi_plane 
);


//! Make odometry computation for the current frame
//! \param  [out]  odometry_multi_plane    The odometry object
//! \param  [in ]  model                   The 3D model
//! \param  [in ]  camera                  The camera used for odometry estimation
//! \param  [in ]  threshold               Minimum score for a plane to be considered successfully tracked
//! \return An error code
//! \todo   To be tested
//! \todo   Do we really need to pass the model ? It has been aleady passed at initialization
ROX_API Rox_ErrorCode rox_odometry_multi_plane_make ( 
   Rox_Odometry_Multi_Plane odometry_multi_plane,
   const Rox_Model_Multi_Plane    model,
   const Rox_Camera               camera
);


//! Retrieve last estimated pose
//! \param  [out]  pose                    The estimated pose of the camera with repect to the 3D object
//! \param  [in ]  odometry_multi_plane    The odometry object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_get_pose ( 
   Rox_MatSE3 pose, 
   const Rox_Odometry_Multi_Plane odometry_multi_plane 
);


//! Retrieve last estimated score
//! \param  [out]  score                   The estimated score
//! \param  [in ]  odometry_multi_plane    The odometry object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_get_score ( 
   Rox_Double * score, 
   const Rox_Odometry_Multi_Plane odometry_multi_plane 
);

//! Retrieve odometry result
//! \param  [out]  score                   The estimated score
//! \param  [in ]  odometry_multi_plane    The odometry object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_get_result ( 
   Rox_Sint * is_tracked, 
   Rox_Double * score, 
   Rox_MatSE3 pose, 
   const Rox_Odometry_Multi_Plane odometry_multi_plane 
);


//! Set the pose inside the multi plane odometry object
//! \param  [out]  odometry       The odometry object
//! \param  [in ]  pose           The estimated pose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_set_pose ( 
   Rox_Odometry_Multi_Plane odometry_multi_plane, 
   const Rox_MatSE3 pose 
);


//! Set the threshold to decide if the odometry is lost
//! \param  [out]  odometry the odometry object
//! \param  [in ]  score_thresh the decision threshold
//! \remark score_thresh must be in the range [0,1]
//! \remark default value is score_thresh = 0.89
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_set_score_threshold (
   Rox_Odometry_Multi_Plane odometry_multi_plane, 
   const Rox_Double score_threshold
);

//! Set the maximum number of iterations
//! \param  [in] odometry the odometry object
//! \param  [in] miter the maximum number of iterations
//! \remarks default value is miter = 10
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_multi_plane_set_miter (
   Rox_Odometry_Multi_Plane odometry, 
   const Rox_Sint miter
);

//! @}

#endif // __OPENROX_ODOMETRY_MULTI_PLANE__
