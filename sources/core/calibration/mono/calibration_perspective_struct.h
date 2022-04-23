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

#ifndef __OPENROX_CALIBRATION_PERSPECTIVE_STRUCT__
#define __OPENROX_CALIBRATION_PERSPECTIVE_STRUCT__

#include <generated/objset_array2d_double.h>
#include <generated/objset_array2d_double_struct.h>
#include <generated/objset_dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_uint_struct.h>

#include <baseproc/geometry/point/points_struct.h>

//! \ingroup Camera_Calibration
//! \defgroup Calibration_Mono_Perspective Perspective Monocular Calibration

//! \addtogroup Calibration_Mono_Perspective
//! @{

//! The Rox_Calibration_Mono_Perspective_Struct object 
struct Rox_Calibration_Mono_Perspective_Struct
{
   //! The 3D coordinates of the model [X, Y, 0, 1]
   Rox_DynVec_Point3D_Double model;

   //! The reduced coordinates of the model [X, Y, 1]
   //! Be careful this is stored in Point2D object but it is NOT a 2D point !!!
   Rox_DynVec_Point2D_Double model2D; // TODO: change the name from model2D to reduced_model

   //! The 2D coordinates of the detected points 
   Rox_ObjSet_DynVec_Point2D_Double points;

   //! The pose set
   Rox_ObjSet_Array2D_Double poses;

   //! The pose set copy
   Rox_ObjSet_Array2D_Double poses_cpy;

   //! The homography set
   Rox_ObjSet_Array2D_Double homographies;

   //! The validity flags
   Rox_DynVec_Uint valid_flags;

   //! The estimated intrinsic parameters
   Rox_MatUT3 K;

   //! A copy of the estimated intrinsic parameters
   Rox_MatUT3 K_cpy;

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

#endif // __OPENROX_CALIBRATION_PERSPECTIVE_STRUCT__
