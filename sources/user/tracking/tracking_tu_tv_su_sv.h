//==============================================================================
//
//    OPENROX   : File tracking_tu_tv_su_sv.h
//
//    Contents  : API of tracking_tu_tv_su_sv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING_TU_TV_SU_SV__
#define __OPENROX_TRACKING_TU_TV_SU_SV__

#include "tracking.h"
#include "tracking_params.h"

#include <core/patch/patchplane_pyramid.h>
#include <core/tracking/patch/tracking_patch_tu_tv_su_sv.h>
#include <core/predict/plane_search.h>

//! \ingroup Tracking
//! \defgroup Tracking_tu_tv_su_sv Tracking tu, tv, su, sv

//!  \addtogroup Tracking_tu_tv_su_sv
//!  @{

//! The Rox_Tracking_tu_tv_su_sv_Struct object 
struct Rox_Tracking_tu_tv_su_sv_Struct
{
   //! The generic tracking structure 
   Rox_Tracking_Struct parent;

   //! The patch pyramid 
   Rox_PatchPlane_Pyramid pyramid;

   //! The plane tracker 
   Rox_Tracking_Patch_tu_tv_su_sv tracker;

   //! The plane predicter 
   Rox_Plane_Search predicter;
};

//! Define the pointer of the Rox_Odometry_M2D_Light_Affine_Struct 
typedef struct Rox_Tracking_tu_tv_su_sv_Struct* Rox_Tracking_tu_tv_su_sv;

//! Create the tracking object
//! \param[out] tracking The tracking parameters
//! \param[in] params   The tracking parameters
//! \param[in] model    The 2d model
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_su_sv_new(Rox_Tracking_tu_tv_su_sv * tracking, Rox_Tracking_Params params, Rox_Image model);

//! Delete the tracking object
//! \param[in] tracking tracking object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_su_sv_del(Rox_Tracking_tu_tv_su_sv * tracking);

//! Make the tracking
//! \param[in] tracking tracking object
//! \param[in] image   the current image
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_su_sv_make(Rox_Tracking_tu_tv_su_sv tracking, Rox_Image image);

//! Set the template mask
//! \param[in] tracking the tracking object
//! \param[in] mask the template mask
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_su_sv_set_mask(Rox_Tracking_tu_tv_su_sv tracking, const Rox_Imask mask);

//! @} 

#endif // __OPENROX_TRACKING_TU_TV_SU_SV__ 
