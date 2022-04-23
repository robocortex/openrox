//==============================================================================
//
//    OPENROX   : File centered_error.h
//
//    Contents  : API of centered_error module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CENTERED_ERROR__
#define __OPENROX_CENTERED_ERROR__

#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>

//! \ingroup Array2D_Float
//! \brief for each element compute A-B-mean(A-B)
//! \param [out] 	median 			The median of the differences
//! \param [out] 	sum_square 		The sum of square difference
//! \param [out] 	count_valid 	The number of non masked pixels
//! \param [out] 	res 				The destination array
//! \param [in]		mask 				The input mask
//! \param [in] 	one 				The left operand
//! \param [in] 	two 				The right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_centered_error(Rox_Double * median, Rox_Double * sum_square, Rox_Uint * count_valid, Rox_Array2D_Float res, Rox_Array2D_Uint mask, Rox_Array2D_Float one, Rox_Array2D_Float two);

#endif // __OPENROX_CENTERED_ERROR__
