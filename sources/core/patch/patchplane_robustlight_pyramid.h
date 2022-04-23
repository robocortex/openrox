//==============================================================================
//
//    OPENROX   : File patchplane_robustlight_pyramid.h
//
//    Contents  : API of patchplane_robustlight_pyramid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PATCHPLANEROBUSTLIGHT_PYRAMID__
#define __OPENROX_PATCHPLANEROBUSTLIGHT_PYRAMID__

#include "patchplane_robustlight.h"

#include <baseproc/image/imask/imask.h>

//!
//! \ingroup Patch
//! \addtogroup PatchPlanePyramid_RobustLight
//! @{

//! The Rox_PatchPlane_RobustLight_Pyramid_Struct object 
struct Rox_PatchPlane_RobustLight_Pyramid_Struct
{
   //! The pyramid size 
   Rox_Uint count;

   //! The patch list 
   Rox_PatchPlane_RobustLight * levels;
};

//! Define the pointer of the Rox_PatchPlane_RobustLight_Pyramid_Struct 
typedef struct Rox_PatchPlane_RobustLight_Pyramid_Struct * Rox_PatchPlane_RobustLight_Pyramid;

//! Create patch plane pyramid object
//! \param  [out] obj the pointer to the patch object
//! \param  [in] height the patch height
//! \param  [in] width the patch width
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_pyramid_new(Rox_PatchPlane_RobustLight_Pyramid * obj, Rox_Sint height, Rox_Sint width);

//! Delete patch plane pyramid object
//! \param  obj the pointer to the patch object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_pyramid_del(Rox_PatchPlane_RobustLight_Pyramid * obj);

//! Apply reference image to pyramid
//! \param  [out] obj the pointer to the patch object
//! \param  [in] source the reference image to add
//! \param  [in] mask the reference mask to add
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_pyramid_apply(Rox_PatchPlane_RobustLight_Pyramid obj, Rox_Array2D_Float source, Rox_Imask mask);

//! Reset luminosity model information
//! \param  [out] obj the patch object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_pyramid_reset_luminance(Rox_PatchPlane_RobustLight_Pyramid obj);

//! @} 

#endif
