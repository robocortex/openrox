//==============================================================================
//
//    OPENROX   : File camproj_calibration.h
//
//    Contents  : API of camproj_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CAMPROJ_CALIBRATION__
#define __OPENROX_CAMPROJ_CALIBRATION__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>

#include <core/calibration/mono/calibration_perspective.h>
#include <core/calibration/projector/calibration_perspective.h>

//! \ingroup  Sensor
//! \defgroup CamProj CamProj
//! \brief Structure and functions for stereo cameras.

//! \ingroup  CamProj
//! \defgroup Calibration_CamProj_Perspective CamProj Camera Calibration
//! \brief Structure and functions for camera calibration.
//! \warning: synthetic tests ran fine with 4 projected points but failed with 9 Debug is ongoing

//! \addtogroup Calibration_CamProj_Perspective
//! \brief Structure and functions of the perspective camproj calibration
//! @{

//! Define the pointer of the Rox_Calibration_CamProj_Perspective_Struct 
typedef struct Rox_Calibration_CamProj_Perspective_Struct * Rox_Calibration_CamProj_Perspective;

//! Create a new calibration object
//! \param  []  obj is the newly created calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_new( Rox_Calibration_CamProj_Perspective * obj );

//! Delete a calibration object
//! \param  []  obj is the created calibration object to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_del( Rox_Calibration_CamProj_Perspective * obj );

//! Set the 3D coordinates of the calibration model used for the camera
//! \param  []  obj     The calibration object to use
//! \param  []  refs3D  The list of the 3D model coordinates
//! \param  [] count   The number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_set_cam_model_points(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Point3D_Double refs3D,
   Rox_Uint count     
);

//! Set the 2D coordinates of the pattern projected by the projector
//! \param  []  obj       The calibration object to use
//! \param  []  refs2D    The list of the 2D pattern coordinates (in pixels)
//! \param  []  count     The number of points in the list
//! \return An error code
 
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_set_proj_model_points(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Point2D_Double refs2D,
   Rox_Uint  count     
);

//! Add the 2D detected points of the two images to the calibration object
//! \param  []  obj               the calibration to use
//! \param  []  cam_pts 
//! \param  []  cam_count 
//! \param  []  proj_pts
//! \param  []  proj_count 
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_add_current_points(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Point2D_Double cam_pts,
   Rox_Uint           cam_count,
   Rox_Point2D_Double proj_pts,
   Rox_Uint           proj_count  
);

//! Add the current 3D-2D homographies (G) of the calibration model
//! \param  []  obj               The calibration to use
//! \param  []  G                 The estimated homography
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_add_current_cam_homography(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_MatSL3                  G     );

//! Initialize the intrinsic parameters for the linear estimation
//! \param  []  obj               The calibration object to use
//! \param  []  K_cam             The initialization of the intrinsic parameters to copy
//! \param  []  K_proj            The initialization of the intrinsic parameters to copy
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_set_intrinsics(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_MatUT3                  K_cam,
   Rox_MatUT3                  K_proj  );

//! Refine the linear estimation of the intrinsic parameters (fu, fv, cu, cv and skew) using a non linear approach (visual servoing to minimalize the projection error)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_process_nolinear(Rox_Calibration_CamProj_Perspective obj);

//! Refine the linear estimation of the intrinsic parameters (f, cu, cv only) using a non linear approach (visual servoing to minimalize the projection error)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_process_nolinear_f_cu_cv(Rox_Calibration_CamProj_Perspective obj);

//! Compute the linear and no linear estimations of the intrinsic parameters
//! \param  []  obj              the calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_make(Rox_Calibration_CamProj_Perspective obj);

//! Set the camera image resolution to initialize the optical centers
//! \param  []  obj               The calibration object to use
//! \param  []  cols              The cols
//! \param  []  rows              The rows
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_set_cam_resolution(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Sint                            cols,
   Rox_Sint                            rows   );

//! Set the projector image resolution to initialize the optical centers
//! \param  []  obj               The calibration object to use
//! \param  []  cols              The cols
//! \param  []  rows              The rows
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_set_proj_resolution(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Sint                            cols,
   Rox_Sint                            rows   );

//! Get a copy of the calibration results, Kp as not refined
//! \param  []  obj              The calibration object to use
//! \param  []  Kc               The camera intrinsic parameters
//! \param  []  Kp               The projector intrinsic parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_get_linear_intrinsics(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_MatUT3                  Kc,
   Rox_MatUT3                  Kp    );

//! Get a copy of the calibration results, Kp as refined
//! \param  []  obj               The calibration object to use
//! \param  []  Kc                The camera intrinsic parameters
//! \param  []  Kp                The projector intrinsic parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_get_refined_intrinsics(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_MatUT3                  Kc,
   Rox_MatUT3                  Kp    );

//! Obtain a relative pose after linear estimation
//! \param  []  obj               The calibration object to use
//! \param  []  pTc               The matrix pose to be filled
//! \param  []  index             The index of the relative pose to pick
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_get_indexed_linear_pTc(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_MatSE3                  pTc,
   Rox_Uint                            index  );

//! Obtain a relative pose after non linear refinement
//! \param  []  obj               The calibration object to use
//! \param  []  pTc               The matrix pose to be filled
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_get_refined_pTc(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_MatSE3                  pTc   );

//! Compute mean reprojection error, once the calibration has been made, for unrefined parameters
//! * (i.e. Kp, pTc )
//! \param  []  obj               The calibration object to use
//! \param  []  error_cam         The mean error for the camera
//! \param  []  error_proj        The mean error for the projector
//! \param  []  index             The index of the camera-projector couple on which errors are to be computed
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_check_linear_results(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Double*                         error_cam,
   Rox_Double*                         error_proj,
   Rox_Uint                            index        );

//! Compute mean and median reprojection error, once the calibration has been made,
//! with refined parameters.
//! For one or all images. If only one image, the median return value should be ignored.
//! \param  []  obj                   The calibration object to use
//! \param  []  error_cam             The mean error for the camera
//! \param  []  mean_error_proj_fwd   The mean error   for camera points transfered    to projector points
//! \param  []  median_error_proj_fwd The median error for camera points transfered    to projector points
//! \param  []  mean_error_proj_bwd   The mean error   for projector points transfered to camera points
//! \param  []  median_error_proj_bwd The median error for projector points transfered to camera points
//! \param  []  index                 if (!= -1): the index of the camera-projector couple on which errors are to be computed. Otherwise all pairs are treated.
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_check_refined_results(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Double*                         error_cam,
   Rox_Double*                         mean_error_proj_fwd,
   Rox_Double*                         median_error_proj_fwd,
   Rox_Double*                         mean_error_proj_bwd,
   Rox_Double*                         median_error_proj_bwd,
   Rox_Sint                            index );

//! Set camera intrinsics, therefore disabling camera calibration
//! \param  []  obj               The calibration object to use
//! \param  []  Kin               The input 3x3 intrinsics matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_set_camera_intrinsics(
   Rox_Calibration_CamProj_Perspective  obj,
   Rox_MatUT3                   Kin   );

//! For each used image, compute and display some reprojection statistics
//! \param  []  obj                  The calibration object to use
//! \return An error code
// ROX_API Rox_ErrorCode rox_calibration_camproj_perspective_print_statistics(Rox_Calibration_CamProj_Perspective obj);

//! @} 

#endif // __OPENROX_CAMPROJ_CALIBRATION__
