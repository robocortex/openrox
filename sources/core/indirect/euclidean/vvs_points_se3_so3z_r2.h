//==============================================================================
//
//    OPENROX   : File vvs_points_se3_so3z_r2.h
//
//    Contents  : API of vvs_se3_so3z_so3z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VVS_POINTS_SE3_SO3Z_R2__
#define __OPENROX_VVS_POINTS_SE3_SO3Z_R2__

#include <baseproc/maths/linalg/matse3.h>
#include <generated/objset_matse3.h>
#include <generated/objset_dynvec_point2d_double.h>
#include <generated/objset_dynvec_point3d_double.h>

//! Visual servoing of points to compute the optimal poses cTo and oTb 
//! \param  [out]  cTo            The pose of the object frame relative to the camera frame
//! \param  [out]  oTb            The poses for each local frame relative to the object frame
//! \param  [in ]  qr             The 2D points observed in the image (in normalized coordinates)
//! \param  [in ]  mb             The 3D points of the model in its local frames
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_vvs_points_nor_se3_so3z_r2 (
   Rox_MatSE3 cTo, 
   Rox_ObjSet_MatSE3 oTb,               // Should be an objset since there are many possible planes
   const Rox_ObjSet_DynVec_Point2D_Double qr,  // Should be an objset of dynvec since there are many possible planes
   const Rox_ObjSet_DynVec_Point3D_Double mb   // Should be an objset of dynvec since there are many possible planes
);

//! Visual servoing of points to compute the optimal poses cTo and oTb 
//! \param  [out]  cTo            The pose of the object frame relative to the camera frame
//! \param  [out]  oTb            The poses for each local frame relative to the object frame
//! \param  [in ]  Kc             The camera calibration parameters
//! \param  [in ]  pr             The 2D points observed in the image (in pixel coordinates)
//! \param  [in ]  mb             The 3D points of the model in its local frames
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_vvs_points_pix_se3_so3z_r2
(
   Rox_MatSE3 cTo, 
   Rox_ObjSet_MatSE3 oTb,                      // Should be an objset since there are many possible planes
   const Rox_Matrix Kc,
   const Rox_ObjSet_DynVec_Point2D_Double pr,  // Should be a dynvec since there are many possible planes
   const Rox_ObjSet_DynVec_Point3D_Double mb   // Should be a dynvec since there are many possible planes
);

#endif
