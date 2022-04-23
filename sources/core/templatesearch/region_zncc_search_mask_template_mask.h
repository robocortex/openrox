//============================================================================
//
//    OPENROX   : File region_zncc_search_mask_template_mask.h
//
//    Contents  : API of region_zncc_search_mask_template_mask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REGION_ZNCC_SEARCH_MASK_TEMPLATE_MASK__
#define __OPENROX_REGION_ZNCC_SEARCH_MASK_TEMPLATE_MASK__

#include <generated/array2d_float.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

// Defin the % of the template that should be visible
#define MIN_VISIBLE_RATIO 0.25 

//! \ingroup Image
//! \addtogroup ZnccSearch
//! @{

//! Search for a given template in an image using ZNCC in a window. 
//! Function may provide random erroneous results for template with more than 2^16-1 pixels
//! \param  [out]  res_score      The best score found in [0,1]
//! \param  [out]  res_topleft_x  The best score x coordinate
//! \param  [out]  res_topleft_y  The best score y coordinate
//! \param  [in ]  isearch        The image to search into
//! \param  [in ]  isearch_mask   The validity mask of isearch
//! \param  [in ]  itemplate      The image patch
//! \param  [in ]  itemplate_mask The image patch validity mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_region_zncc_search_mask_template_mask (
   Rox_Float * res_score, 
   Rox_Sint * res_topleft_x, 
   Rox_Sint * res_topleft_y, 
   const Rox_Array2D_Float isearch, 
   const Rox_Imask isearch_mask, 
   const Rox_Array2D_Float itemplate, 
   const Rox_Imask itemplate_mask
);

//! Search for a given template in an image using ZNCC in a window. 
//! Function may provide random erroneous results for template with more than 2^16-1 pixels
//! \param  [out]  res_score      The best score found in [0,1]
//! \param  [out]  res_topleft_x  The best score x coordinate
//! \param  [out]  res_topleft_y  The best score y coordinate
//! \param  [in ]  isearch        The image to search into
//! \param  [in ]  isearch_mask   The validity mask of isearch
//! \param  [in ]  itemplate      The image patch
//! \param  [in ]  itemplate_mask The image patch validity mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_region_zncc_search_mask_template_mask (
   Rox_Float * res_score, 
   Rox_Sint * res_topleft_x, 
   Rox_Sint * res_topleft_y, 
   const Rox_Image isearch, 
   const Rox_Imask isearch_mask, 
   const Rox_Image itemplate, 
   const Rox_Imask itemplate_mask
);

//! @} 

#endif // __OPENROX_REGION_ZNCC_SEARCH_MASK_TEMPLATE_MASK__
