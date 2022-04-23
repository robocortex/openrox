//==============================================================================
//
//    OPENROX   : File ransacessentialpose.h
//
//    Contents  : API of ransacessentialpose module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANSAC_ESSENTIAL_POSE__
#define __OPENROX_RANSAC_ESSENTIAL_POSE__

#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_sint.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \ingroup Geometry
//! \addtogroup Essential
//! @{

//! Robustly estimate pose matrix from points in two views using ransac algorithm (fischer 1981)
//! \param  [out]  pose           computed pose
//! \param  [in ]  calib          internal calibration of the camera
//! \param  [out]  inlierscur     destination points which are not violated by the model
//! \param  [out]  inliersref     source points which are not violated by the model
//! \param  [in ]  cur            destination points in meters
//! \param  [in ]  ref            source points in meters
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ransac_essential_pose (
   Rox_MatSE3 pose, 
   Rox_MatUT3 calib, 
   Rox_DynVec_Point2D_Float inlierscur, 
   Rox_DynVec_Point2D_Float inliersref, 
   Rox_DynVec_Point2D_Float cur, 
   Rox_DynVec_Point2D_Float ref
);

//! Given model parameters, Compute the inliers/outliers
//! \param  [out]  cardconsensus  number of inliers
//! \param  [out]  inliers        a mask with 1 for inliers, 0 for outliers
//! \param  [in ]  pose           the estimated pose between reference and current
//! \param  [in ]  calib          the camera intrinsics
//! \param  [in ]  cur2D          source points in meters (same size than ref2d)
//! \param  [in ]  ref2D          destination points in meters (same size than cur2d)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_essential_check_3Dconsensus (
   Rox_Uint * cardconsensus, 
   Rox_Uint * inliers, 
   Rox_Array2D_Double pose, 
   Rox_Array2D_Double calib, 
   Rox_DynVec_Point2D_Float ref2D, 
   Rox_DynVec_Point2D_Float cur2D
);

//! Using geometric constraint, extract from a set of pose (computed from E) the unique possible pose.
//! \param  [out]  idpose         the best pose id among the set of possible poses
//! \param  [in ]  poses          the set of possible poses
//! \param  [in ]  refs           destination points in meters (same size than cur2d)
//! \param  [in ]  curs           source points in meters (same size than ref2d)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_essential_check_poses (
   Rox_Uint * idpose, 
   Rox_MatSE3 * poses, 
   Rox_Point2D_Double  refs, 
   Rox_Point2D_Double  curs
);

//! @}

#endif
