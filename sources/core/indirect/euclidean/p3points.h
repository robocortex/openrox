//==============================================================================
//
//    OPENROX   : File p3points.h
//
//    Contents  : API of p3points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_P3POINTS__
#define __OPENROX_P3POINTS__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point3d.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \ingroup Geometry
//! \addtogroup p3p
//! @{

//! Estimate the transformation between reference and current frame using known 3 pairs of 3D points in both frames
//! \param  [out]  cTr                Estimated pose
//! \param  [in ]  ctriangle          Array of three points in 3D current frame
//! \param  [in ]  rtriangle          Array of three points in 3D reference frame
//! \return An error code
//! \todo   To be tested. This function should not be here but in a geometry module and named "rox_matse3_from_p3p"
ROX_API Rox_ErrorCode rox_pose_from_two_3D_triangles (
   Rox_MatSE3 cTr, 
   Rox_Point3D_Double ctriangle, 
   Rox_Point3D_Double rtriangle
);

//! Estimate pose between reference and current frame
//! \param  [out]  poses                list of possible poses
//! \param  [out]  validposes           count possible poses (max 4)
//! \param  [in ]  points3D             list of 3d points in reference frame
//! \param  [in ]  points2D             list of 2d points in current frame
//! \param  [in ]  fu                   camera calibration parameter
//! \param  [in ]  fu                   camera calibration parameter
//! \param  [in ]  cu                   camera calibration parameter
//! \param  [in ]  cv                   camera calibration parameter
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pose_from_3_points (
   Rox_Array2D_Double_Collection poses, 
   Rox_Uint * validposes, 
   Rox_Point3D_Double points3D, 
   Rox_Point2D_Double points2D, 
   double fu, 
   double fv, 
   double cu, 
   double cv
);

//! Estimate pose solving the P3p problem
//! \param  [out]  poses                list of possible poses
//! \param  [out]  validposes           count possible poses (max 4)
//! \param  [in ]  points3D             Array of three 3D points (meters) in reference frame
//! \param  [in ]  points2D             Array of three 2D points (normal) in current frame
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pose_from_p3p_nor (
   Rox_Array2D_Double_Collection poses, 
   Rox_Uint * validposes, 
   Rox_Point3D_Double points3D, 
   Rox_Point2D_Double points2D
);

//! Estimate pose solving the P3p problem
//! \param  [out]  possible_poses       List of possible poses
//! \param  [out]  valid_poses          Count possible poses (max 4)
//! \param  [in ]  points3D_met         Array of three 3d points in reference frame
//! \param  [in ]  points2D_pix         Array of three 2d points in current frame
//! \param  [in ]  calib                Camera calibration parameter
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pose_from_p3p_pix (
   Rox_Array2D_Double_Collection possible_poses, 
   Rox_Uint * valid_poses, 
   Rox_Point3D_Double points3D_met, 
   Rox_Point2D_Double points2D_pix, 
   Rox_MatUT3 calib
);

//! @}

#endif
