//==============================================================================
//
//    OPENROX   : File median.h
//
//  	Contents  : API of medianrow module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MEDIAN__
#define __OPENROX_MEDIAN__

#include <generated/array2d_double.h>

//! \ingroup Statistics
//! \addtogroup Median
//! @{

//! Compute the median of given vector.
//! Array will be modified in this function. Make a copy if necessary.
//! \param [out] median a pointer to the computed median
//! \param [in] input the vector to compute median on
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_median(Rox_Double * median, Rox_Array2D_Double input);

//! Sort a vector in ascending order
//! \param [in] input the vector to compute median on
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_asort(Rox_Array2D_Double input);

//! @} 

#endif // __OPENROX_MEDIAN__
