//============================================================================
//
//    OPENROX   : File remap_box_mask_halved.h
//
//    Contents  : API of remap_box_mask_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REMAPHALVEDMASK__
#define __OPENROX_REMAPHALVEDMASK__

#include <baseproc/image/imask/imask.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Resize a mask to half its original dimensions (a mask cell is set to valid if the four original cells are valid).
//! \param  [out]  imask_out      The resized mask (created with half the width and height of source mask)
//! \param  [in ]  imask_inp      The original mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_remap_halved_mask ( 
   Rox_Array2D_Uint imask_out, 
   Rox_Array2D_Uint imask_inp
);

//! @}

#endif
