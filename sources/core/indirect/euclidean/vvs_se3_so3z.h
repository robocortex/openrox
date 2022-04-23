//==============================================================================
//
//    OPENROX   : File vvs_se3_so3z.h
//
//    Contents  : API of vvs_se3_so3z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VVS_SE3_SO3Z__
#define __OPENROX_VVS_SE3_SO3Z__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>

#include "vvspointsse3.h" // <- some tool functions are in there

//! \addtogroup Servoing
//! @{

//! Given:
//! . two lists of 3D points, each in its own frame,
//! . two lists of associated 2D points measurements,
//! . An a priori transformation from the first frame to the second
//!    with an unknown rotation in the XY plane
//! . An priori pose estimation relatively to the right frame
//! minimize the reprojection error by looking for:
//! . a valid pose
//! . a valid correction of the frame to frame transformation
//! 
//! Visual virtual servoing approach (linear approx.)
//! \param  [out]  cTr		      The pose result and input
//! \param  [out]  Tr   	      The rotation around z axis
//! \param  [in ]  qr   	      The right points 2D observed pixels (normalized coordinates)
//! \param  [in ]  mr   	      The right points 3D reference
//! \param  [in ]  ql   	      The left points 2D observed pixels (normalized coordinates)
//! \param  [in ]  ml   	      The left points 3D reference
//! \param  [in ]  maxdist_prefilter max distance for a point otherwise it is prefiltered in normalized units
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_float_refine_pose_vvs_se3_so3z (
	Rox_Array2D_Double        cTr,
	Rox_Array2D_Double        Tr,
	Rox_DynVec_Point2D_Float  qr,
	Rox_DynVec_Point3D_Float  mr,
	Rox_DynVec_Point2D_Float  ql,
	Rox_DynVec_Point3D_Float  ml,
	Rox_Double          maxdist_prefilter 
);

//! @} 

#endif
