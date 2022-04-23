//==============================================================================
//
//    OPENROX   : File triangulate.h
//
//    Contents  : API of triangulate module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRIANGULATE__
#define __OPENROX_TRIANGULATE__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Geometry
//! \addtogroup Triangulate
//! @{

//! Triangulate 3d coordinates of a point given 2 measurements using the simplest method (Intersection of three planes).
//! Note that this method is NOT to be used when the measurements are not perfectly related by the "pose essential matrix"
//! See Nister article appendix in An Efficient Solution to the Five-Point Relative Pose Problem
//! \param  []  result            The pointer to the result 3D point
//! \param  []  ref2D             The measurement in the reference frame
//! \param  []  cur2D             The measurement in the current frame
//! \param  []  pose              The pose of the current frame in the reference frame.
//! \return An error code
ROX_API Rox_ErrorCode rox_pose_triangulate_simple (
   Rox_Point3D_Double result, 
   const Rox_Point2D_Double ref2D, 
   const Rox_Point2D_Double cur2D, 
   const Rox_MatSE3 pose
);

//! Triangulate 3d coordinates of a point given 2 measurements by minimizing the current reprojection.
//! \param  [out]  result         The pointer to the result 3D point in the reference frame
//! \param  [in ]  ref2D          The measurement in the reference frame
//! \param  [in ]  cur2D          The measurement in the current frame
//! \param  [in ]  pose           The pose of the current frame in the reference frame (cur_T_ref).
//! \return An error code
ROX_API Rox_ErrorCode rox_pose_triangulate_oneway (
   Rox_Point3D_Double result, 
   const Rox_Point2D_Double ref2D, 
   const Rox_Point2D_Double cur2D, 
   const Rox_MatSE3 pose
);

//! Correct pair of 2D points optimally such that they lie on the epipolar line.
//! See Triangulation Made Easy by Peter Lindstrom for detailed description.
//! \param  [out]  corref         The pointer to the result 3D point
//! \param  [out]  corcur         The pointer to the result 3D point
//! \param  [in ]  ref2D          The measurement in the reference frame
//! \param  [in ]  cur2D          The measurement in the current frame
//! \param  [in ]  pose           the pose of the current frame in the reference frame.
//! \return An error code
ROX_API Rox_ErrorCode rox_pose_triangulate_correction_gold ( 
   Rox_Point2D_Double corref, 
   Rox_Point2D_Double corcur, 
   const Rox_Point2D_Double ref2D, 
   const Rox_Point2D_Double cur2D, 
   const Rox_MatSE3 pose
);

//! @} 

#endif
