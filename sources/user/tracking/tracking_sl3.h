//==============================================================================
//
//    OPENROX   : File tracking_sl3.h
//
//    Contents  : API of tracking_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING_SL3__
#define __OPENROX_TRACKING_SL3__

#include "tracking.h"
#include "tracking_params.h"
#include <core/patch/patchplane_pyramid.h>
#include <core/tracking/patch/tracking_patch_sl3.h>
#include <core/predict/plane_search.h>

//! \ingroup Tracking
//! \defgroup Tracking_SL3 Tracking MatSL3

//! \addtogroup Tracking_SL3
//! @{

//! The Rox_Tracking_SL3_Struct object

struct Rox_Tracking_SL3_Struct
{
   //! The generic tracking structure 
   Rox_Tracking_Struct parent;

   //! The patch pyramid 
   Rox_PatchPlane_Pyramid pyramid;

   //! The plane tracker 
   Rox_Tracking_Patch_SL3 tracker;

   //! The plane predicter 
   Rox_Plane_Search predicter;
};

//! Define the pointer of the Rox_Odometry_M2D_Light_Affine_Struct 
typedef struct Rox_Tracking_SL3_Struct* Rox_Tracking_SL3;

//! \brief Create the tracking object
//! \param [out]  tracking    The tracking parameters
//! \param [in]   params      The tracking parameters
//! \param [in]   model       The 2d model
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_sl3_new(Rox_Tracking_SL3 * tracking, Rox_Tracking_Params params, Rox_Image model);

//! \brief Delete the tracking object
//! \param [in]   tracking tracking object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_sl3_del(Rox_Tracking_SL3 * tracking);

//! \brief Make the tracking
//! \param [in]   tracking    Tracking object
//! \param [in]   image       The current image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_sl3_make(Rox_Tracking_SL3 tracking, const Rox_Image image);

//! \brief Set the template mask
//! \param [in]   tracking    The tracking object
//! \param [in]   mask        The template mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_sl3_set_mask(Rox_Tracking_SL3 tracking, const Rox_Imask mask);

//! @} 

#endif // __OPENROX_TRACKING_SL3__ 
