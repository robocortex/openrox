//==============================================================================
//
//    OPENROX   : File stereo_calibration.h
//
//    Contents  : API of stereo_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_STEREO_CALIBRATION__
#define __OPENROX_STEREO_CALIBRATION__


#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/point/point3d.h>

#include <core/calibration/mono/calibration_perspective.h>

//! \ingroup  Sensor
//! \defgroup Stereo Stereo
//! \brief Structure and functions for stereo cameras.

//! \ingroup  Stereo
//! \defgroup Calibration_Stereo_Perspective Stereo Camera Calibration
//! \brief Structure and functions for camera calibration.

//! \addtogroup Calibration_Stereo_Perspective
//! \brief Structure and functions of the perspective stereo calibration
//! @{

//! Define the pointer of the Rox_Calibration_Stereo_Perspective_Struct 
typedef struct Rox_Calibration_Stereo_Perspective_Struct * Rox_Calibration_Stereo_Perspective;

//! Create a new calibration object
//! \param  []  obj               The newly created calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_new(Rox_Calibration_Stereo_Perspective * obj);

//! Delete a calibration object
//! \param  []  obj               The created calibration object to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_del(Rox_Calibration_Stereo_Perspective * obj);

//! Set the 3D coordinates of the calibration model
//! \param  []  obj               The calibration object to use
//! \param  []  pts               The list of the 3D model coordinates
//! \param  []  count             The number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_set_model_points(Rox_Calibration_Stereo_Perspective obj, Rox_Point3D_Double pts, Rox_Uint count);

//! Add the 2D detected points of the two images to the calibration object
//! \param  []  obj               The calibration to use
//! \param  []  left_pts          The list of points with 2D information filled (2D in pixels)
//! \param  []  right_pts         The list of points with 2D information filled (2D in pixels)
//! \param  []  count             The number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_add_current_points(Rox_Calibration_Stereo_Perspective obj, Rox_Point2D_Double left_pts, Rox_Point2D_Double right_pts, Rox_Uint count);

//! Add the current 3D-2D homographies (G) of the calibration model
//! \param  []  obj               The calibration to use
//! \param  []  Gl                The left image estimated homography
//! \param  []  Gr                The right image estimated homography
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_add_current_homographies(Rox_Calibration_Stereo_Perspective obj, Rox_MatSL3 Gl, Rox_MatSL3 Gr);

//! Initialize the intrinsic parameters for the linear estimation
//! \param  []  obj               The calibration object to use
//! \param  []  Kl                The initialization of the intrinsic parameters to copy
//! \param  []  Kr                The initialization of the intrinsic parameters to copy
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_set_intrinsics(Rox_Calibration_Stereo_Perspective obj, Rox_MatUT3 Kl, Rox_MatUT3 Kr);

//! Refine the linear estimation of the intrinsic parameters (fu, fv, cu, cv and skew) using a non linear approach (visual servoing to minimalize the projection error)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_process_nolinear(Rox_Calibration_Stereo_Perspective obj);

//! Refine the linear estimation of the intrinsic parameters (f, cu, cv only) using a non linear approach (visual servoing to minimalize the projection error)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_process_nolinear_f_cu_cv(Rox_Calibration_Stereo_Perspective obj);

//! Compute the linear and no linear estimations of the intrinsic parameters
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_make(Rox_Calibration_Stereo_Perspective obj);

//! Set the image resolution to initialize the optical centers
//! \param  []  obj               The calibration object to use
//! \param  []  cols              The cols
//! \param  []  rows              The rows
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_set_resolution(Rox_Calibration_Stereo_Perspective obj, Rox_Sint cols, Rox_Sint rows);

//! Get a copy of the calibration results
//! \param  []  Kl                The left intrinsic parameters
//! \param  []  Kr                The right intrinsic parameters
//! \param  []  pose              The stereo pose
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_get_results(Rox_MatUT3 Kl, Rox_MatUT3 Kr, Rox_MatSE3 pose, Rox_Calibration_Stereo_Perspective obj);

//! For each used image, compute and display some reprojection statistics
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_print_statistics(Rox_Calibration_Stereo_Perspective obj);

//! Initialize the stereo pose after the linear estimation of the parameters
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_stereo_perspective_compute_intercamera_pose(Rox_Calibration_Stereo_Perspective obj);

//! @} 

#endif // __OPENROX_STEREO_CALIBRATION__
