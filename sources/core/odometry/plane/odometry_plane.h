//==============================================================================
//
//    OPENROX   : File odometry_plane.h
//
//    Contents  : API of odometry_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_PLANE__
#define __OPENROX_ODOMETRY_PLANE__

#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <core/patch/patchplane.h>

//! \ingroup Odometry
//! \defgroup Odometry_Plane Odometry Plane

//! \addtogroup Odometry_Plane
//! @{

//! Define the pointer of the Rox_Odometry_Plane_Struct
typedef struct Rox_Odometry_Plane_Struct * Rox_Odometry_Plane;

//! Create tracking plane object
//! \param  [out] obj the pointer to the tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_new ( Rox_Odometry_Plane * obj);

//! Delete tracking plane object
//! \param  [in]  obj                     the pointer to the tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_del ( Rox_Odometry_Plane * obj);

//! Perform tracking on the given patch
//! \param  [out]  obj                    The pointer to the tracking object
//! \param  [in ]  patch                  The patch to track
//! \param  [in ]  source                 The captured image to track into
//! \param  [in ]  max_iters              The maximum iteration count (if no convergence)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_make (
   Rox_Odometry_Plane obj, 
   const Rox_PatchPlane patch, 
   const Rox_Array2D_Float source, 
   const Rox_Sint max_iters
);

//! Set plane pose + calibration
//! \param  [out]  obj                    The tracking object
//! \param  [in ]  pose                   The pose matrix
//! \param  [in ]  calibration_camera     The calibration matrix for camera
//! \param  [in ]  calibration_template   The calibration matrix for the template
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_set_pose (
   Rox_Odometry_Plane obj, 
   Rox_MatSE3 pose, 
   Rox_MatUT3 calibration_camera, 
   Rox_MatUT3 calibration_template
);

//! Get plane pose
//! \param  [out]  pose                   The pose matrix
//! \param  [in ]  obj                    The tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_plane_get_pose (
   Rox_MatSE3 pose, 
   Rox_Odometry_Plane obj
);

//! @}

#endif
