//==============================================================================
//
//    OPENROX   : File odometry_plane_struct.h
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

#ifndef __OPENROX_ODOMETRY_PLANE_STRUCT__
#define __OPENROX_ODOMETRY_PLANE_STRUCT__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <core/patch/patchplane.h>


//! \addtogroup Odometry
//! @{

//! The Rox_Odometry_Plane_Struct object
struct Rox_Odometry_Plane_Struct
{
   //! The result Hessian matrix    (J'*J)
   Rox_Array2D_Double JtJ;

   //! The result projected vector  (J'*diff)
   Rox_Array2D_Double Jtf;

   //! The inverse of (J'*J)
   Rox_Array2D_Double iJtJ;

   //! The solution vector (pose + illumation changes)
   Rox_Array2D_Double solution;
   
   //! The solution vector (only pose)
   Rox_Array2D_Double solution_pose;

   //! The 3D pose
   Rox_Array2D_Double pose;

   //! The intrinsic parameters
   Rox_Array2D_Double calibration_camera;

   //! The 3D-2D calibration of the template
   Rox_Array2D_Double calibration_template;

   //! The homography matrix
   Rox_Array2D_Double homography;
};

//! @}

#endif
