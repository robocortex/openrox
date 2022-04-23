//==============================================================================
//
//    OPENROX   : File stereo_calibration_se3_struct.h
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

#ifndef __OPENROX_STEREO_CALIBRATION_SE3_STRUCT__
#define __OPENROX_STEREO_CALIBRATION_SE3_STRUCT__

#include <generated/array2d_double.h>
#include <generated/objset_array2d_double.h>
#include <generated/objset_array2d_double_struct.h>
#include <generated/objset_dynvec_point2d_double_struct.h>
#include <generated/dynvec_uint_struct.h>

#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_point2d_double_struct.h>

//! \ingroup  Stereo
//! \defgroup Calibration_Stereo_Perspective_SE3 Stereo Camera Calibration_SE3
//! \brief Structure and functions for camera calibration.

//! \addtogroup Calibration_Stereo_Perspective_SE3
//! \brief Structure and functions of the perspective stereo calibration
//! @{

//! The Rox_Calibration_Stereo_Perspective_SE3_Struct object 
struct Rox_Calibration_Stereo_Perspective_SE3_Struct
{
   //! The 3D coordinates of the model 
   Rox_DynVec_Point3D_Double model;

   //! The left 2D coordinates of the model 
   Rox_ObjSet_DynVec_Point2D_Double lpoints;

   //! The right 2D coordinates of the model 
   Rox_ObjSet_DynVec_Point2D_Double rpoints;

   //! The left pose set 
   Rox_ObjSet_Array2D_Double lposes;

   //! The right pose set 
   Rox_ObjSet_Array2D_Double rposes;

   //! The left homography set 
   Rox_ObjSet_Array2D_Double lhomographies;

   //! The right homography set 
   Rox_ObjSet_Array2D_Double rhomographies;

   //! The left intrinsic parameters 
   Rox_Array2D_Double Kl;

   //! The right intrinsic parameters 
   Rox_Array2D_Double Kr;

   //! The inter camera pose (calibration result)
   Rox_Array2D_Double rTl;

   //! The inter camera pose set 
   Rox_ObjSet_Array2D_Double rTlposes;

   //! The valid flags 
   Rox_DynVec_Uint valid_flags;
};

//! @} 

#endif // __OPENROX_STEREO_CALIBRATION_SE3_STRUCT__
