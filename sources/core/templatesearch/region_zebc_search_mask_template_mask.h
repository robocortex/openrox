//============================================================================
//
//    OPENROX   : File region_zebc_search_mask_template_mask.h
//
//    Contents  : API of the template matching module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REGION_ZEBC_SEARCH_MASK_TEMPLATE_MASK__
#define __OPENROX_REGION_ZEBC_SEARCH_MASK_TEMPLATE_MASK__

#include <generated/array2d_float.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Image
//! \addtogroup ZnccSearch
//! @{

//! Search for a given template in an image using ZEBC in a window. Function may provide random erroneous results for template with more than 2^16-1 pixels
//! \param  [out]  res_score      The best score found in [0,1]
//! \param  [out]  res_u          The best u coordinate shift
//! \param  [out]  res_v          The best v coordinate shift
//! \param  [in ]  isearch        The image to search into
//! \param  [in ]  isearch_mask   The validity mask of isearch
//! \param  [in ]  itemplate      The image patch
//! \param  [in ]  itemplate_mask The image patch validity mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_region_zebc_search_mask_template_mask (
   Rox_Float * res_score, 
   Rox_Sint * res_u, 
   Rox_Sint * res_v, 
   const Rox_Array2D_Float I, 
   const Rox_Imask I_mask, 
   const Rox_Array2D_Float T, 
   const Rox_Imask T_mask, 
   const Rox_Float zncc_threshold
);

//! @} 

#endif // __OPENROX_REGION_ZEBC_SEARCH_MASK_TEMPLATE_MASK__
