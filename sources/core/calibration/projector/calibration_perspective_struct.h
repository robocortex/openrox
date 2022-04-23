//==============================================================================
//
//    OPENROX   : File calibration_perspective_struct.h
//
//    Contents  : API of calibration_perspective module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_PROJECTOR_PERSPECTIVE_STRUCT__
#define __OPENROX_CALIBRATION_PROJECTOR_PERSPECTIVE_STRUCT__

#include <baseproc/geometry/point/points_struct.h>
#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/objset_array2d_double_struct.h>
#include <generated/objset_dynvec_point2d_double.h>
#include <generated/objset_dynvec_point3d_double.h>

//! \ingroup Projector_Calibration
//! \defgroup Calibration_Projector_Perspective Perspective Projector Calibration

//! \addtogroup Calibration_Projector_Perspective
//! @{

//! The Rox_Calibration_Projector_Perspective_Struct object
struct Rox_Calibration_Projector_Perspective_Struct
{
   //! The 3D coordinates of the model
	Rox_ObjSet_DynVec_Point3D_Double obs3D;

   //! The 2D coordinates of the model
   Rox_ObjSet_DynVec_Point2D_Double obs2D;

   //! The 2D coordinates of the detected points
   Rox_DynVec_Point2D_Double ref2D;

   //! The pose set
   Rox_ObjSet_Array2D_Double poses;

   //! The pose set copy
   Rox_ObjSet_Array2D_Double poses_cpy;

   //! The homography set
   Rox_ObjSet_Array2D_Double homographies;

   //! The validity flags
   Rox_DynVec_Uint valid_flags;

   //! The estimated intrinsic parameters
   Rox_Array2D_Double K;

   //! A copy of the estimated intrinsic parameters
   Rox_Array2D_Double K_cpy;

   //! The image counter
   Rox_Uint valid_image;

   // Buffers
   //! Working buffer 
   Rox_Array2D_Double A0;
   //! Working buffer 
   Rox_Array2D_Double A1;
   //! Working buffer 
   Rox_Array2D_Double A2;
   //! Working buffer 
   Rox_Array2D_Double A3;
   //! Working buffer 
   Rox_Array2D_Double A4;
   //! Working buffer 
   Rox_Array2D_Double A5;

   //! Working buffer 
   Rox_Array2D_Double C0;
   //! Working buffer 
   Rox_Array2D_Double C1;
   //! Working buffer 
   Rox_Array2D_Double C2;
   //! Working buffer 
   Rox_Array2D_Double C3;
   //! Working buffer 
   Rox_Array2D_Double C4;
   //! Working buffer 
   Rox_Array2D_Double C5;

   //! Working buffer 
   Rox_Array2D_Double M;
   //! Working buffer 
   Rox_Array2D_Double MtA;
   //! Working buffer 
   Rox_Array2D_Double Kn;
   //! Working buffer 
   Rox_Array2D_Double Kt;
   //! Working buffer 
   Rox_Array2D_Double S;

};

//! @} 

#endif // __OPENROX_CALIBRATION_PROJECTOR_PERSPECTIVE_STRUCT__
