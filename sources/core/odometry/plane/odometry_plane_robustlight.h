//==============================================================================
//
//    OPENROX   : File odometry_plane_robustlight.h
//
//    Contents  : API of odometry_plane_robustlight module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_PLANE_ROBUST_LIGTH__
#define __OPENROX_ODOMETRY_PLANE_ROBUST_LIGTH__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <core/patch/patchplane_robustlight.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \addtogroup Odometry
//! @{

//! Define the pointer of the Rox_Odometry_Plane_RobustLight_Struct */
typedef struct Rox_Odometry_Plane_RobustLight_Struct * Rox_Odometry_Plane_RobustLight;

//! Create tracking plane object
//! \param [out] obj the pointer to the tracking object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_robustlight_new(Rox_Odometry_Plane_RobustLight * obj);

//! Delete tracking plane object
//! \param [in] obj the pointer to the tracking object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_robustlight_del(Rox_Odometry_Plane_RobustLight * obj);

//! Perform tracking on the given patch
//! \param obj the pointer to the tracking object
//! \param patch the patch to track
//! \param source the captured image to track into
//! \param max_iters the maximum iteration count (if no convergence)
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_robustlight_make(Rox_Odometry_Plane_RobustLight obj, Rox_PatchPlane_RobustLight patch, Rox_Array2D_Float source, Rox_Sint max_iters);

//! Set plane pose + calibration
//! \param obj the tracking object
//! \param pose the pose matrix
//! \param calibration_camera the calibration matrix for camera
//! \param calibration_template the calibration matrix for the template
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_robustlight_set_pose(Rox_Odometry_Plane_RobustLight obj, Rox_MatSE3 pose, Rox_MatUT3 calibration_camera, Rox_MatUT3 calibration_template);

//! Get plane pose
//! \param pose the pose matrix
//! \param obj the tracking object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_robustlight_get_pose(Rox_Array2D_Double pose, Rox_Odometry_Plane_RobustLight obj);

//! @}
#endif
