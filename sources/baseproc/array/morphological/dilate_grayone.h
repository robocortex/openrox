//==============================================================================
//
//    OPENROX   : File dilate_grayone.h
//
//    Contents  : API of dilate_grayone module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DILATEGRAYONE__
#define __OPENROX_DILATEGRAYONE__

#include <generated/array2d_float.h>

//! \ingroup Image
//! \addtogroup Morphological
//! @{

//! Grayscale dilatation with a 3*3 kernel
//! \param [in]   res      the result image
//! \param [in]   input    the original image
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_float_dilate_grayone(Rox_Array2D_Float res, Rox_Array2D_Float input);

//! @} 

#endif
