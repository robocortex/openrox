//============================================================================
//
//    OPENROX   : File odometry_se2.h
//
//    Contents  : API of odometry_se2 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ODOMETRY_SE2__
#define __OPENROX_ODOMETRY_SE2__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <core/patch/patchplane.h>

//! \ingroup Odometry
//! \defgroup Odometry_SE2 Odometry MatSE2

//! \addtogroup Odometry_SE2
//! @{

//! The Rox_Odometry_Plane_Struct object
struct Rox_Odometry_SE2_Struct
{
   //! The result Hessian Matrix  (J^t*J)
   Rox_Array2D_Double JtJ;

   //! The result projected vector (J^t*diff) 
   Rox_Array2D_Double Jtf;

   //! The inverse of (J^t*J) 
   Rox_Array2D_Double iJtJ;

   //! The solution vector (pose + illumation changes) 
   Rox_Array2D_Double solution;

   //! The solution vector (only pose) 
   Rox_Array2D_Double solution_pose;

   //! The 3D pose full result
   Rox_Array2D_Double pose;

   //! The 3D pose 
   Rox_Array2D_Double pose_se2;

   //! The intrinsic parameters 
   Rox_Array2D_Double calibration_camera;

   //! The homography matrix 
   Rox_Array2D_Double homography;
};

//! Define the pointer of the Rox_Odometry_Plane_Struct 
typedef struct Rox_Odometry_SE2_Struct * Rox_Odometry_SE2;

//! Create tracking se2 object
//! \param  [out] obj the pointer to the tracking object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_se2_new(Rox_Odometry_SE2 * obj);

//! Delete tracking se2 object
//! \param  [out] obj the pointer to the tracking object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_se2_del(Rox_Odometry_SE2 * obj);

//! Perform tracking on the given patch
//! \param  [out] obj the pointer to the tracking object
//! \param  [in] patch the patch to track
//! \param  [in] source the captured image to track into
//! \param  [in] max_iters the maximum iteration count (if no convergence)
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_se2_make(Rox_Odometry_SE2 obj, Rox_PatchPlane patch, Rox_Array2D_Float source, Rox_Uint max_iters);

//! Get pose
//! \param [out] pose the pose matrix
//! \param [in] obj the tracking object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_odometry_se2_get_pose(Rox_Array2D_Double pose, Rox_Odometry_SE2 obj);

//! Set calibration
//! \param [out] obj the tracking object
//! \param [in] calibration_camera calibration to set
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_odometry_se2_set_calib(Rox_Odometry_SE2 obj, Rox_Array2D_Double calibration_camera);

//! @} 

#endif
