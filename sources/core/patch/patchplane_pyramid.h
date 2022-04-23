//==============================================================================
//
//    OPENROX   : File patchplane_pyramid.h
//
//    Contents  : API of patchplane_pyramid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PATCHPLANE_PYRAMID__
#define __OPENROX_PATCHPLANE_PYRAMID__

#include "patchplane.h"

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Patch
//! \addtogroup PatchPlanePyramid
//! @{

//! The Rox_PatchPlane_Pyramid_Struct object 
struct Rox_PatchPlane_Pyramid_Struct
{
   //! The pyramid size 
   Rox_Uint count;

   //! The patch list 
   Rox_PatchPlane * levels;
};

//! Define the pointer of the Rox_PatchPlane_Pyramid_Struct 
typedef struct Rox_PatchPlane_Pyramid_Struct * Rox_PatchPlane_Pyramid;

//! Create patch plane pyramid object
//! \param  [out] patchplane_pyramid         The pointer to the patch object
//! \param  [in]  height                     The patch height
//! \param  [in]  width                      The patch width
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_pyramid_new(Rox_PatchPlane_Pyramid * patchplane_pyramid, Rox_Sint height, Rox_Sint width);

//! Delete patch plane pyramid object
//! \param [out] patchplane_pyramid          The pointer to the patch object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_pyramid_del(Rox_PatchPlane_Pyramid * patchplane_pyramid);

//! Apply reference image to pyramid
//! \param  [out] patchplane_pyramid         The pointer to the patch object
//! \param  [in]  source                     The reference image to add
//! \param  [in]  mask                       The reference mask to add (if NULL the mask is not applied)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_pyramid_apply(Rox_PatchPlane_Pyramid patchplane_pyramid, Rox_Array2D_Float source, Rox_Imask mask);

//! Reset luminosity model information
//! \param  [out] patchplane_pyramid         The patch object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_pyramid_reset_luminance(Rox_PatchPlane_Pyramid patchplane_pyramid);

//! @} 

#endif
