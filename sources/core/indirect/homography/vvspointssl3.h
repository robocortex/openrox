//==============================================================================
//
//    OPENROX   : File vvspointssl3.h
//
//    Contents  : API of vvspointssl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VVS_POINTS_SL3__
#define __OPENROX_VVS_POINTS_SL3__

#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>

#include <baseproc/maths/linalg/matsl3.h>

//! \ingroup Geometry
//! \addtogroup Homography
//! @{

//! Given a list of reference 2D points and associated 2D points measurements, minimize the reprojection error by looking for a valid homography.
//! Visual virtual servoing approach (linear approx.)
//! \param  [out]  homography     The homography result and input
//! \param  [in ]  measures       The 2D observed pixels (in pixels)
//! \param  [in ]  references     The 2D reference pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_float_refine_homography_vvs (
   Rox_MatSL3 homography, 
   const Rox_DynVec_Point2D_Float measures, 
   const Rox_DynVec_Point2D_Float references
);

//! @} 

#endif
