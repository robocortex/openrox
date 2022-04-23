//==============================================================================
//
//    OPENROX   : File vvs_se3_so3z_so3z.h
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

#ifndef __OPENROX_VVS_SE3_SO3Z_SO3Z__
#define __OPENROX_VVS_SE3_SO3Z_SO3Z__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include "vvspointsse3.h" // <- some tool functions are in there

//! \addtogroup Servoing
//! @{

// Illustration
//
//            P                                                       G
//                      z                                    z    y
//                    /                                        \ //
//                   /__ x                                  x __\//
//                  //
//                  //                                               
//                   y                                             \ z
//                                                                  \___ x
//                                                                  //
//         y    z                                                   //       B
//          // /                                                      y
//       ___///
//     x
//              S
//
//                                     z
//                                      /
//                                     /____ x
//                                     //
//                                     //
//                                   y       C

//! Given:
//!
//! Assumes:
//! . we have 4 frames: p(etit), g(rand), s(mall), b(ig)
//! . the transformations sTp and bTg are only made of:
//! . a translation along Z (p/g frame)
//! . a rotation in the XY plane (p/g frame)
//! 
//! . two lists of 3D points, each in its own frame ( s(mall) and b(ig) )
//! . two lists of associated 2D points measurements,
//! . An a priori transformation from the frame P to the frame G => gTp
//! . An unknown XY plane (i.e. around Z) rotation component in the transformation pTs
//! . An unknown XY plane (i.e. around Z) rotation component in the transformation bTg
//! . An a priori camera pose estimation relatively to the B frame => cTb
//!
//! The transformation for points from the small set to the camera frame is:
//!   Mc = cTb . bTg . gTp . pTs . Ms
//!
//! The transformation for points from the big set to the camera frame is:
//!   Mc = cTb . Mb
//!
//!  The function minimizes the reprojection error refining:
//! . cTb
//! . gTp XY plane rotation component only
//! . pTs XY plane rotation component only
//!
//!   Visual virtual servoing approach (linear approx.)
//! \param  [out]  cTb  the approximate pose to refine
//! \param  [out]  bTg  the rotation around z axis for the second frame to refine
//! \param  [out]  pTs  the rotation around z axis for the first frame to refine
//! \param  [in ]  gTp  the a priori transformation between the two frames
//! \param  [in ]  qb   the big points 2D observed pixels (normalized coordinates)
//! \param  [in ]  mb   the big points 3D reference
//! \param  [in ]  qs   the small points 2D observed pixels (normalized coordinates)
//! \param  [in ]  ms   the small points 3D reference
//! \param  [in ]  maxdist_prefilter max distance for a point otherwise it is prefiltered, expressed in normalized units
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_float_refine_pose_vvs_se3_so3z_so3z (
   Rox_Array2D_Double       cTb,
   Rox_Array2D_Double       bTg,
   Rox_Array2D_Double       pTs,
   Rox_Array2D_Double       gTp,
   Rox_DynVec_Point2D_Float qb,
   Rox_DynVec_Point3D_Float mb,
   Rox_DynVec_Point2D_Float qs,
   Rox_DynVec_Point3D_Float ms,
   Rox_Double               maxdist_prefilter 
);

//! @} 

#endif
