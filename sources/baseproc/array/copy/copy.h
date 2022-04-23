//==============================================================================
//
//    OPENROX   : File copy.h
//
//    Contents  : API of copy module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_COPY__
#define __OPENROX_COPY__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

//! \ingroup  Array2D
//! \defgroup Array2D_Double Array2D_Double
//! \brief Functions for array of type double.

//! \ingroup  Array2D
//! \defgroup Array2D_Float Array2D_Float
//! \brief Functions for array of type float.

//! \ingroup Array2D_Double
//! @{

//! Copy a 2D array into a row 2D array
//! \param  [out]  res            the result
//! \param  [in ]  one            the left operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_vector_row_copy(Rox_Array2D_Double vR, Rox_Array2D_Double R);

//! @} 

//! \ingroup Array2D_Float
//! @{

//! Copy a 2D array into a col 2D array
//! \param  [out]  res            the result
//! \param  [in ]  one            the left operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_vector_col_copy(Rox_Array2D_Double vR, Rox_Array2D_Double R);

//! @} 

#endif
