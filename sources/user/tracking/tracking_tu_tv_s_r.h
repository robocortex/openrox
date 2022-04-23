//==============================================================================
//
//    OPENROX   : File tracking_tu_tv_s_r.h
//
//    Contents  : API of tracking_tu_tv_s_r module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING_TU_TV_S_R__
#define __OPENROX_TRACKING_TU_TV_S_R__

#include "tracking.h"
#include "tracking_params.h"

#include <core/patch/patchplane_pyramid.h>
#include <core/tracking/patch/tracking_patch_tu_tv_s_r.h>
#include <core/predict/plane_search.h>

//! \ingroup Tracking
//! \defgroup Tracking_tu_tv_s_r Tracking tu, tv, s, r

//! \addtogroup Tracking_tu_tv_s_r
//! @{

//! The Rox_Tracking_tu_tv_s_r_Struct object 
struct Rox_Tracking_tu_tv_s_r_Struct
{
   //! The generic tracking structure 
   Rox_Tracking_Struct parent;

   //! The patch pyramid 
   Rox_PatchPlane_Pyramid pyramid;

   //! The plane tracker 
   Rox_Tracking_Patch_tu_tv_s_r tracker;

   //! The plane predicter 
   Rox_Plane_Search predicter;
};

//! Define the pointer of the Rox_Odometry_M2D_Light_Affine_Struct 
typedef struct Rox_Tracking_tu_tv_s_r_Struct* Rox_Tracking_tu_tv_s_r;

//! Create the tracking object
//! \param [out]  tracking    The tracking parameters
//! \param [in]   params      The tracking parameters
//! \param [in]   model       The 2d model
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_s_r_new(Rox_Tracking_tu_tv_s_r * tracking, Rox_Tracking_Params params, Rox_Image model);

//! Delete the tracking object
//! \param [in]   tracking    The tracking object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_s_r_del(Rox_Tracking_tu_tv_s_r * tracking);

//! Make the tracking
//! \param [in]   tracking    The tracking object
//! \param [in]   image       The current image
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_s_r_make(Rox_Tracking_tu_tv_s_r tracking, Rox_Image image);

//! Set the template mask
//! \param [in]   tracking    The tracking object
//! \param [in]   mask        The template mask
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_tracking_tu_tv_s_r_set_mask(Rox_Tracking_tu_tv_s_r tracking, const Rox_Imask mask);

//! @} 

#endif // __OPENROX_TRACKING_tu_tv_s_r__ 
