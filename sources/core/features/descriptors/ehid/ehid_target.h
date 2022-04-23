//==============================================================================
//
//    OPENROX   : File ehid_target.h
//
//    Contents  : API of ehid_target module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_TARGET__
#define __OPENROX_EHID_TARGET__

#include <generated/dynvec_ehid_point.h>
#include <generated/array2d_double.h>

//! \addtogroup EHID
//! @{

//! Target structure for identification 
typedef struct Rox_Ehid_Target_Struct * Rox_Ehid_Target;

//! Create a new target object
//! \param [out] obj the pointer of the newly created target
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_new(Rox_Ehid_Target * obj);

//! Delete a target object
//! \param [out] obj the pointer of the deleted target
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_del(Rox_Ehid_Target * obj);

//! Reset a target to initial state (not found, no match)
//! \param [in] obj the target to reset
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_reset(Rox_Ehid_Target obj);

//! Compute statistics for a target, mostly estimating the best viewpoint
//! \param [in] obj the target to use
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_compute_stats(Rox_Ehid_Target obj);

//! Remove the repeating features from the matching list
//! \param [in] obj the target to use
//! \param [in] detectedfeats the list of runtime features
//! \param [in] globaldb the list of reference features
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_cleanup(Rox_Ehid_Target obj, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb);

//! Estimate the pose of a target using current matchs
//! \param [in] obj the target object to use
//! \param [in] detectedfeats the list of runtime features
//! \param [in] globaldb the list of reference features
//! \param [in] calib_output the camera intrinsics
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_estimate_pose(Rox_Ehid_Target obj, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb, Rox_Array2D_Double calib_output);

//! Estimate the homography of a target using current matchs
//! \param [in] obj the target object to use
//! \param [in] detectedfeats the list of runtime features
//! \param [in] globaldb the list of reference features
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_estimate_homography(Rox_Ehid_Target obj, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb);

//! Tell the target to ignore all matchs of the best viewpoints for further identification in this frame
//! \param [in] obj the target object
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_ignorebestvp(Rox_Ehid_Target obj);

//! Tell the target to ignore all matchs of all viewpoints for further identification in this frame
//! \param [in] obj the target object
//! \return An error code
ROX_API Rox_ErrorCode rox_ehid_target_ignoreallvp(Rox_Ehid_Target obj);

//! @} 

#endif
