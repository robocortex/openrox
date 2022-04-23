//==============================================================================
//
//    OPENROX   : File mad.h
//
//    Contents  : API of madrow module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MADROW__
#define __OPENROX_MADROW__

#include <generated/array2d_double.h>

//! \ingroup Statistics
//! \addtogroup MAD
//!   @{

//! Compute the median absolute deviation of the first row of the given array.
//! Array will be modified in this function. Make a copy if necessary.
//! \param [out] 	mad 				A pointer to the computed MAD
//! \param [out] 	adfrommedian	An array which will store the absolue deviation from median (same size than input)
//! \param [out] 	workbuffer		An array which will store temporary values (same size than input)
//! \param [in] 	input				The array to compute MAD on
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_mad(Rox_Double * mad, Rox_Array2D_Double adfrommedian, Rox_Array2D_Double workbuffer, Rox_Array2D_Double input);

//! @} 

#endif // __OPENROX_MADROW__
