//==============================================================================
//
//    OPENROX   : File ransacsl3.h
//
//    Contents  : API of ransacsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANSAC_SL3__
#define __OPENROX_RANSAC_SL3__

#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_sint.h>

#include <baseproc/maths/linalg/matsl3.h>

//! \ingroup Geometry
//! \addtogroup Homography
//! @{

//! Robustly estimate homography from points in two views using ransac algorithm (fischer 1981)
//! \param  [out]  homography    computed homography
//! \param  [out]  inlierscur    destination points which are not violated by the model
//! \param  [out]  inliersref    source points which are not violated by the model
//! \param  [in ]  cur           destination points
//! \param  [in ]  ref           source points
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_ransac_homography (
   Rox_MatSL3 homography, 
   Rox_DynVec_Point2D_Float inlierscur, 
   Rox_DynVec_Point2D_Float inliersref, 
   Rox_DynVec_Point2D_Float cur, 
   Rox_DynVec_Point2D_Float ref
);

//! @} 

#endif
