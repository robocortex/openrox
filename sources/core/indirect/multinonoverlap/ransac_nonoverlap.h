//==============================================================================
//
//    OPENROX   : File ransac_nonoverlap.h
//
//    Contents  : API of ransac_nonoverlap module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANSAC_NONOVERLAP__
#define __OPENROX_RANSAC_NONOVERLAP__

#include <generated/array2d_double.h>
#include <generated/objset_dynvec_point2d_double.h>

//! \ingroup Geometry
//! \addtogroup NonOverlap
//! @{

//! Find an optimal pose from a noisy set of points using Ransac in multiple cameras
//! \param [out] pose         the estimated pose of the camera rig
//! \param [in] relativeposes the relative poses of the camera rig (wrt the common frame) for each camera of the rig
//! \param [in] calibrations  the intrinsic parameters for each camera of the rig
//! \param [in] refs          a set of vectors with matched reference points
//! \param [in] curs          a set of vectors with matched current points
//! \return An error code
ROX_API Rox_ErrorCode rox_ransac_nonoverlap(Rox_Array2D_Double pose, Rox_Array2D_Double_Collection relativeposes, Rox_Array2D_Double_Collection calibrations, Rox_ObjSet_DynVec_Point2D_Double refs, Rox_ObjSet_DynVec_Point2D_Double curs);

//! @} 

#endif
