//==============================================================================
//
//    OPENROX   : File stereo_calibration_struct.h
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

#ifndef __OPENROX_CAMPROJ_CALIBRATION_STRUCT__
#define __OPENROX_CAMPROJ_CALIBRATION_STRUCT__

#include <generated/objset_array2d_double.h>
#include <baseproc/geometry/point/point2d.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>

#include <core/calibration/projector/calibration_perspective.h>
#include <core/calibration/projector/calibration_perspective_struct.h>
#include <core/calibration/mono/calibration_perspective_struct.h>
#include <core/calibration/mono/calibration_perspective.h>


//! \addtogroup Calibration_CamProj_Perspective
//! \brief Structure and functions of the perspective stereo calibration
//! @{

//! Structure 
struct Rox_Calibration_CamProj_Perspective_Struct
{
   //! The camera calibration object 
   Rox_Calibration_Mono_Perspective cam;

   //! The projector calibration object 
   Rox_Calibration_Projector_Perspective proj;

   //! The linearly estimated pTc pose set 
   Rox_ObjSet_Array2D_Double poses;

   //! The refined pTc pose 
   Rox_MatSE3 pTc;

   //! The refined projector intrinsics 
   Rox_MatUT3 Kp;

   //! The valid flags 
   // Rox_DynVec_Uint valid_flags;

   //! Temporary stored points corresponding to the projected
   //! pattern detected by the camera
   Rox_ObjSet_DynVec_Point2D_Double camproj_pts2D;

   //! The projector tracking homography 
   Rox_MatSL3 Gp;

   //! The point list of the projected grid 
   Rox_Point2D_Double proj_refs2D;

   //! The number of projected points 
   Rox_Uint nproj_pts;

   //! Flag indicating whether we provided the intrinsic parameters or not 
   Rox_Uint known_kc;

   //! Camera intrinsic parameters matrix
   Rox_MatUT3 Kc;

   //! The camera pose set 
   Rox_ObjSet_Array2D_Double cTos;

};

//! @} 

#endif // __OPENROX_CAMPROJ_CALIBRATION_STRUCT__
