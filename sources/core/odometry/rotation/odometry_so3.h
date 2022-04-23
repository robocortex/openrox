//============================================================================
//
//    OPENROX   : File odometry_so3.h
//
//    Contents  : API of odometry_so3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ODOMETRY_SO3__
#define __OPENROX_ODOMETRY_SO3__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <core/patch/patchplane.h>

//! \ingroup Odometry
//! \defgroup Odometry_SO3 Odometry MatSO3

//! \addtogroup Odometry_SO3
//! @{

//! The Rox_Odometry_SO3_Struct object 
struct Rox_Odometry_SO3_Struct
{
   //! The result Hessian Matrix  (J^t*J)
   Rox_Array2D_Double JtJ;

   //! The result projected vector (J^t*diff) 
   Rox_Array2D_Double Jtf;

   //! The intermediate Hessian Matrix  (J^t*J)
   Rox_Array2D_Double bJtJ;

   //! The intermediate projected vector (J^t*diff) 
   Rox_Array2D_Double bJtf;

   //! The inverse of (J^t*J) 
   Rox_Array2D_Double iJtJ;

   //! The solution vector (pose + illumation changes) 
   Rox_Array2D_Double solution;

   //! The solution vector (only pose) 
   Rox_Array2D_Double solution_pose;

   //! The 3D pose full result
   Rox_Array2D_Double pose;

   //! The 3D pose right camera buffer
   Rox_Array2D_Double pose_right;

   //! The 3D pose 
   Rox_Array2D_Double pose_so3;

   //! The intrinsic parameters 
   Rox_Array2D_Double calibration_camera;

   //! The homography matrix 
   Rox_Array2D_Double homography;
};

//! Define the pointer of the Rox_Odometry_SO3_Struct 
typedef struct Rox_Odometry_SO3_Struct * Rox_Odometry_SO3;

//! Create tracking so3 object
//! \param  [out]  obj the pointer to the tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_so3_new(Rox_Odometry_SO3 * obj);

//! Delete tracking so3 object
//! \param  [in ]  obj the pointer to the tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_so3_del(Rox_Odometry_SO3 * obj);

//! Perform tracking on the given patch
//! \param  []  obj the pointer to the tracking object
//! \param  []  patch the patch to track
//! \param  []  source the captured image to track into
//! \param  []  max_iters the maximum iteration count (if no convergence)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_so3_make (Rox_Odometry_SO3 obj, Rox_PatchPlane patch, Rox_Array2D_Float source, Rox_Sint max_iters);

//! Perform tracking on the given patch with a stero pair
//! \param  []  obj the pointer to the tracking object
//! \param  []  patch the patch to track
//! \param  []  source_left the captured left image to track into
//! \param  []  source_right the captured left image to track into
//! \param  []  pose_lr the left to right relative pose
//! \param  []  max_iters the maximum iteration count (if no convergence)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_so3_make_stereo(Rox_Odometry_SO3 obj, Rox_PatchPlane patch, Rox_Array2D_Float source_left, Rox_Array2D_Float source_right, Rox_Array2D_Double pose_lr, Rox_Sint max_iters);

//! Get pose
//! \param  []  pose the pose matrix
//! \param  []  obj the tracking object
//! \return []  An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_so3_get_pose ( Rox_Array2D_Double pose, Rox_Odometry_SO3 obj);

//! Set calibration
//! \param  []  obj the tracking object
//! \param  []  calibration_camera calibration to set
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_so3_set_calib ( Rox_Odometry_SO3 obj, Rox_Array2D_Double calibration_camera);

//! @} 

#endif
