//==============================================================================
//
//    OPENROX   : File vvspointsse3_stereo.h
//
//    Contents  : API of vvspointsse3_stereo module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VVS_POINTS_SE3_STEREO__
#define __OPENROX_VVS_POINTS_SE3_STEREO__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_double.h>

//! \addtogroup Servoing
//! @{

//! Given a list 3D points and associated 2D points stereo measurements, minimize the reprojection error by looking for a valid pose. Visual virtual servoing approach (linear approx.)
//! \param  [out]  pose           The pose result and input
//! \param  [in ]  calib_left 	 The left camera calibration
//! \param  [in ]  calib_right 	 The right camera calibration
//! \param  [in ]  rTl 				 Stereo extrinsic parameters
//! \param  [in ]  measures_left  The 2D observed pixels (in pixels)
//! \param  [in ]  measures_right The 2D observed pixels (in pixels)
//! \param  [in ]  references 	 The 3D reference pixels
//! \param  [in ]  levels 			 The per point pyramid level
//! \param  [in ]  weights 		 The per point weight
//! \param  [in ]  maxdist_prefilter Maximum distance for a point before it is prefiltered
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_points_float_refine_pose_vvs_stereo(Rox_Array2D_Double pose, Rox_Array2D_Double calib_left, Rox_Array2D_Double calib_right, Rox_Array2D_Double rTl, Rox_DynVec_Point2D_Float measures_left, Rox_DynVec_Point2D_Float measures_right, Rox_DynVec_Point3D_Float references, Rox_DynVec_Sint levels, Rox_DynVec_Double weights, Rox_Double maxdist_prefilter);

//! @} 

#endif
