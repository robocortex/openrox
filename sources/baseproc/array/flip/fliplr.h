//==============================================================================
//
//    OPENROX   : File flip_lr.h
//
//    Contents  : API of flip_lr module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FLIP_LR__
#define __OPENROX_FLIP_LR__

//! \ingroup Array2D
//! \addtogroup Flip
//! @{

#include <generated/array2d_double.h>

//! Flip a matrix using a vertical axis of symmetry (reverse the order of columns)
//! \param [out] output    The flipped array2d
//! \param [in] input      The array2d to flip
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_copy_flip_lr(Rox_Array2D_Double output, Rox_Array2D_Double input);

//! @} 

#endif // __OPENROX_FLIP_LR__
