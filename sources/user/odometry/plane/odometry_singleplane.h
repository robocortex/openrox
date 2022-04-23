//==============================================================================
//
//    OPENROX   : File odometry_singleplane.h
//
//    Contents  : API of odometry_singleplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE__
#define __OPENROX_ODOMETRY_SINGLE_PLANE__

#include "odometry_singleplane_params.h"

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/imask/imask.h>

#include <core/model/model_single_plane.h>

#include <user/sensor/camera/camera.h>


//! \ingroup Odometry
//! \addtogroup Odometry_Single_Plane
//! \brief Structure and functions of the model 2D based odometry
//! @{

//! Define the pointer of the Rox_Odometry_Single_Plane_Struct
typedef struct Rox_Odometry_Single_Plane_Struct * Rox_Odometry_Single_Plane;

//! Define the Rox_Odometry_Single_Plane structure
typedef struct Rox_Odometry_Single_Plane_Struct Rox_Odometry_Single_Plane_Struct;

//! Create the odometry object for localization with respect to the 2D model
//! \param  [out]  odometry       The newly created object
//! \param  [in ]  params         The odometry parameters
//! \param  [in ]  model          The 2d model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_new (
   Rox_Odometry_Single_Plane * odometry, 
   const Rox_Odometry_Single_Plane_Params params, 
   const Rox_Model_Single_Plane model
);

//! Delete the odometry object
//! \param  [out]  odometry odometry object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_del (
   Rox_Odometry_Single_Plane * odometry
);

//! Make the odometry
//! \param  [out]  odometry      The odometry object
//! \param  [in ]  camera        Camera object containing the current image and the intrinsic parameters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_make ( 
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Camera camera 
);

//! Get the odometry score
//! \param  [out]  score          the tracking score
//! \param  [in ]  odometry       the odometry object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_get_score (
   Rox_Double * score, 
   const Rox_Odometry_Single_Plane odometry
);

//! Get the pose representing the transformation of the model frame relative to the camera frame
//! \param  [out]  pose           Pose matrix cTo in SE3
//! \param  [in ]  odometry       The odometry object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_get_pose (
   Rox_MatSE3 pose, 
   const Rox_Odometry_Single_Plane odometry
);

//! Set the prediction pose cTm representing the transformation of the model frame relative to the camera frame
//! \param  [out]  odometry       The odometry object
//! \param  [in ]  pose           The pose cTm representing the transformation of the model frame relative to the camera frame
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_set_pose (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_MatSE3 pose
);

//! Set the threshold to decide if the odometry is lost
//! \param  [out]  odometry       the odometry object
//! \param  [in ]  score_thresh   the decision threshold in the range [0,1]
//! \remark Default value is score_thresh = 0.89
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_set_score_thresh (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Double score_thresh
);

//! Set the maximum number of iterations
//! \param  [out]  odometry       the odometry object
//! \param  [in ]  miter          the maximum number of iterations
//! \remarks default value is miter = 10
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_set_miter (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Sint miter
);

//! \brief Set the template mask
//! \param  [out]  odometry       the odometry object
//! \param  [in ]  mask           the template mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_set_mask (
   Rox_Odometry_Single_Plane odometry, 
   const Rox_Imask mask
);

//! Tell whether the model is fully visible or not, given the current pose and camera
//! \param  [out]  visibility     0 if the model is not fully visible, !=0 if it is fully visible
//! \param  [in ]  odometry       odometry object
//! \param  [in ]  camera         object containing the current image and the intrinsic parameters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_get_model_visibility (
   Rox_Sint                        * visibility,
   const Rox_Odometry_Single_Plane   odometry,
   const Rox_Camera                  camera 
);

//! Allocate the internal data of the structure odometry
//! \param  [out]  odometry      The newly created object
//! \param  [in ]  params        The odometry parameters
//! \param  [in ]  model         The 2d model (image plus dimensions)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_alloc (
   Rox_Odometry_Single_Plane odometry, 
   Rox_Odometry_Single_Plane_Params params, 
   Rox_Model_Single_Plane model
   );

//! Release the allocated memory
//! \param  [in] odometry The structure to be freed
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_free(Rox_Odometry_Single_Plane odometry);

//! @}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE__
