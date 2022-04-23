//==============================================================================
//
//    OPENROX   : File inverse.h
//
//    Contents  : API of inverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INVERSE__
#define __OPENROX_INVERSE__

#include <generated/array2d_float.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Compute the array containing the inverse of all entries of the input array 
//! \param  [out]  out           The output array 
//! \param  [in ]  inp           The input  array 
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_inverse ( Rox_Array2D_Float out, const Rox_Array2D_Float inp );

//! @}

#endif
