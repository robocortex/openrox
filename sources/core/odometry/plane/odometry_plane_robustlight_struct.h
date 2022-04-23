//==============================================================================
//
//    OPENROX   : File odometry_plane_robustlight_struct.h
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

#ifndef __OPENROX_ODOMETRY_PLANE_ROBUST_LIGTH_STRUCT__
#define __OPENROX_ODOMETRY_PLANE_ROBUST_LIGTH_STRUCT__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <core/patch/patchplane_robustlight.h>

//! \addtogroup Odometry
//! @{

//! The Rox_Odometry_Plane_RobustLight_Struct object
struct Rox_Odometry_Plane_RobustLight_Struct
{
   //! The result Hessian Matrix  (J^t*J)
   Rox_Array2D_Double lJtJ;
   //! The result projected vector (J^t*diff) 
   Rox_Array2D_Double lJtf;

   //! The 3D pose 
   Rox_Array2D_Double pose;
   //! The intrinsic parameters 
   Rox_Array2D_Double calibration_camera;
   //! The 3D-2D calibration of the global template 
   Rox_Array2D_Double calibration_template;
   //! The 3D-2D calibration of the local template 
   Rox_Array2D_Double calibration_template_block;
   //! The homography matrix 
   Rox_Array2D_Double homography;
};

//! @} 

#endif
