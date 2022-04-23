//==============================================================================
//
//    OPENROX   : File stereo_calibration_se3.h
//
//    Contents  : API of stereo_calibration_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_STEREO_CALIBRATION_SE3__
#define __OPENROX_STEREO_CALIBRATION_SE3__

#include <generated/objset_dynvec_point2d_double.h>
#include <generated/dynvec_uint.h>
#include <generated/dynvec_point3d_double.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>

//! \ingroup  Stereo
//! \defgroup Calibration_Stereo_Perspective_SE3 Stereo Camera Calibration_SE3
//! \brief Structure and functions for camera calibration.

//! \addtogroup Calibration_Stereo_Perspective_SE3
//! \brief Structure and functions of the perspective stereo calibration
//! @{

//! \brief Define the pointer of the Rox_Calibration_Stereo_Perspective_SE3_Struct 
typedef struct Rox_Calibration_Stereo_Perspective_SE3_Struct * Rox_Calibration_Stereo_Perspective_SE3;

//! Create a new calibration object
//! \param  []  obj               The newly created calibration object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_new (
   Rox_Calibration_Stereo_Perspective_SE3 * obj
);

//! Delete a calibration object
//! \param  []  obj               The created calibration object to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_del (
   Rox_Calibration_Stereo_Perspective_SE3 * obj
);

//! Set the calibration pattern defined by its 3D coordinates
//! \param  []  obj               The object to set
//! \param  []  model             The 3D points of the calibration pattern
//! \param  []  count             The point number
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_set_model_points (
   Rox_Calibration_Stereo_Perspective_SE3 obj, 
   Rox_Point3D_Double  model, 
   Rox_Uint count
);

//! Set the detected points of the calibration pattern
//! \param  []  obj               The object to set
//! \param  []  left_pts          The 2D points in the left image
//! \param  []  right_pts         The 2D points in the right image
//! \param  []  count             The point number
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_add_current_points (
   Rox_Calibration_Stereo_Perspective_SE3 obj, 
   Rox_Point2D_Double  left_pts, 
   Rox_Point2D_Double  right_pts, 
   Rox_Uint count
);

//! Set the current homographies
//! \param  []  obj               The object to set
//! \param  []  Gl                The left image - 3D model transformation
//! \param  []  Gr                The right image - 3D model transformation
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_add_current_homographies (
   Rox_Calibration_Stereo_Perspective_SE3 obj, 
   Rox_MatSL3 Gl, 
   Rox_MatSL3 Gr
);

//! Set the intrinsic parameters for each camera
//! \param  []  obj               The object to set
//! \param  []  Kl                The left camera intrinsic parameters
//! \param  []  Kr                The right camera intrinsic parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_set_intrinsics (
   Rox_Calibration_Stereo_Perspective_SE3 obj, 
   Rox_MatUT3 Kl, 
   Rox_MatUT3 Kr
);

//! Refine the pose estimation using a no linear approach
//! \param  []  obj               The calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_process_nolinear (
   Rox_Calibration_Stereo_Perspective_SE3 obj
);

//! Make the intercamera pose estimation
//! \param  []  obj is the calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_make (
   Rox_Calibration_Stereo_Perspective_SE3 obj
);

//! Get a copy of the estimated pose between the two cameras
//! \param  []  pose              The calibration result
//! \param  []  obj               The calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_get_results (
   Rox_MatSE3 pose, 
   Rox_Calibration_Stereo_Perspective_SE3 obj
);

//! For each used image, reproject the 3D model using the homography and compute reprojection errors to valid the homography
//! \param  []  obj               The calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_check_homographies (
   Rox_Calibration_Stereo_Perspective_SE3 obj
);

//! For each used image, compute and display some reprojection statistics
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_print_statistics (
   Rox_Calibration_Stereo_Perspective_SE3 obj
);

//! Initialize the stereo pose after the linear estimation of the parameters
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_compute_intercamera_pose (
   Rox_Calibration_Stereo_Perspective_SE3 obj
);

//! For a given image, get reprojection statistics
//! \param  []  min 		          The mimnumin of the reprojection errors
//! \param  []  max 		          The maximum of the reprojection errors
//! \param  []  mean 	          The median of the reprojection errors
//! \param  []  median 	          The median of the reprojection errors
//! \param  []  std 		          The standard deviation of the reprojection errors
//! \param  []  obj 		          The calibration object to use
//! \param  []  id 		          The id
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_get_left_statistics(
   Rox_Double *min, 
   Rox_Double *max, 
   Rox_Double *mean, 
   Rox_Double *median, 
   Rox_Double *std, 
   Rox_Calibration_Stereo_Perspective_SE3 obj, 
   Rox_Uint id
);

//! For a given image, get reprojection statistics
//! \param  []  min 		          The mimnumin of the reprojection errors
//! \param  []  max 		          The maximum of the reprojection errors
//! \param  []  mean		          The median of the reprojection errors
//! \param  []  median 	          The median of the reprojection errors
//! \param  []  std 		          The standard deviation of the reprojection errors
//! \param  []  obj 		          The calibration object to use
//! \param  []  id	  	          The id
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_se3_get_right_statistics (
   Rox_Double *min, 
   Rox_Double *max, 
   Rox_Double *mean, 
   Rox_Double *median, 
   Rox_Double *std, 
   Rox_Calibration_Stereo_Perspective_SE3 obj, 
   Rox_Uint id
);

//! @} 

#endif // __OPENROX_STEREO_CALIBRATION_SE3__
