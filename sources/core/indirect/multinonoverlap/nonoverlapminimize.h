//==============================================================================
//
//    OPENROX   : File nonoverlapminimize.h
//
//    Contents  : API of nonoverlapminimize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_NONOVERLAP_MINIMIZE__
#define __OPENROX_NONOVERLAP_MINIMIZE__

#include <generated/array2d_double.h>
#include <generated/objset_dynvec_point2d_double.h>

//! \ingroup Geometry
//! \addtogroup NonOverlap
//! @{

//! Find an optimal pose from a noisy set of points using NLE in multiple cameras
//! \param  [out]  pose          The estimated pose of the camera rig
//! \param  [in ]  relativeposes The relative poses of the camera rig (wrt the common frame) for each camera of the rig
//! \param  [in ]  calibrations  The intrinsic parameters for each camera of the rig
//! \param  [in ]  refs          A set of vectors with matched reference points
//! \param  [in ]  curs          A set of vectors with matched current points
//! \return An error code
ROX_API Rox_ErrorCode rox_nonoverlap_minimize(Rox_Array2D_Double pose, Rox_Array2D_Double_Collection relativeposes, Rox_Array2D_Double_Collection calibrations, Rox_ObjSet_DynVec_Point2D_Double refs, Rox_ObjSet_DynVec_Point2D_Double curs);

//! @} 

#endif
