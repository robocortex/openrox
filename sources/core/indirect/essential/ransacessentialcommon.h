//==============================================================================
//
//    OPENROX   : File ransacessentialcommon.h
//
//    Contents  : API of ransacessentialcommon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANSAC_ESSENTIAL_COMMON__
#define __OPENROX_RANSAC_ESSENTIAL_COMMON__

#include <generated/dynvec_point2d_float.h>
#include <generated/array2d_double.h>

//! \ingroup Geometry
//! \addtogroup Essential
//! @{

//! Select 5 points from a set of points
//! \param  [out]  idxs           The result selected indices
//! \param  [in ]  ref            The reference vector of points
//! \param  [in ]  cur            The current vector of points
//! \param  [in ]  pool           A pool for unique selection
//! \param  [in ]  fu             fu calibration value
//! \param  [in ]  fv             fv calibration value
//! \param  [in ]  cu             cu calibration value
//! \param  [in ]  cv             cv calibration value
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_match_float_select_random_5points (
   Rox_Uint * idxs, 
   Rox_DynVec_Point2D_Float ref, 
   Rox_DynVec_Point2D_Float cur, 
   Rox_Uint * pool, 
   Rox_Double fu, 
   Rox_Double fv, 
   Rox_Double cu, 
   Rox_Double cv
);

//! Build two vector of points which are current and reference inliers for a given set
//! \param  [out]  refinliers     The reference vector of inliers
//! \param  [out]  curinliers     The current vector of inliers
//! \param  [in ]  inliers        The mask for the input vectors of points (1 for inliers, 0 for outliers)
//! \param  [in ]  cur2D          The vector with the input current points
//! \param  [in ]  ref2D          The vector with the input reference points
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_build_essential_inliers_subset (
   Rox_DynVec_Point2D_Float refinliers, 
   Rox_DynVec_Point2D_Float curinliers, 
   Rox_Uint * inliers, 
   Rox_DynVec_Point2D_Float cur2D, 
   Rox_DynVec_Point2D_Float ref2D
);

//! @} 

#endif
