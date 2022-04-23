//============================================================================
//
//    OPENROX   : File pyramid_tools.h
//
//    Contents  : API of pyramid_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_PYRAMID__
#define __OPENROX_PYRAMID__

#include <generated/array2d_float.h>

//! \ingroup Image
//! \defgroup Pyramid Pyramid

//! \addtogroup Pyramid
//! @{

//! Compute optimal pyramid level count
//! \param  [out] bestsize pointer to optimal pyramid level count (the base level is not considered).
//! \param  [in] originalwidth original image width
//! \param  [in] originalheight original image height
//! \param  [in] minimalsize minimal size for the last level
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_pyramid_compute_optimal_level_count( 
   Rox_Uint *bestsize, const Rox_Sint originalwidth, const Rox_Sint originalheight, const Rox_Uint  minimalsize );

//! Build a scale space for a given resolution and an input
//! \param  [out] output_scalespace the result scale space allocated inside the function.
//! \param  [in] source the image to convert to scale space. The image range is assumed to be [0;1].
//! \param  [in] nblevels the number of levels desired in scale space.
//! \param  [in] sigma the initial sigma value.
//! \param  [in] cutoff the cutoff applied to kernel.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_build_scale_space_new( 
   Rox_Array2D_Float_Collection *output_scalespace,
   Rox_Array2D_Float             source,
   Rox_Uint                      nblevels,
   Rox_Float                     sigma,
   Rox_Float                     cutoff );

//! @}

#endif // __OPENROX_PYRAMID__
