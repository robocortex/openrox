//==============================================================================
//
//    OPENROX   : File array2d_double_from_float.h
//
//  	Contents  : API of array2d_double_from_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_DOUBLE_FROM_FLOAT__
#define __OPENROX_DOUBLE_FROM_FLOAT__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \ingroup Array2D
//! \defgroup Data_Conversion Data Conversion
//! \brief Types (re)definititon.

//! \ingroup  Data_Conversion
//! \defgroup Float2Double Flat2Double
//! \brief Conversion from Float to Double.

//! \addtogroup Uchar2Double
//! @{

//! Convert an array from uchar type to double type
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_float(
   Rox_Array2D_Double output, Rox_Array2D_Float input);

//! Convert an array from uchar type to double type and scale values to [0;1]
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_float_normalize(
   Rox_Array2D_Double output, Rox_Array2D_Float input);

//! Convert an array from uchar type to double type and scale values to [0;1] using min max values
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_float_normalize_minmax(
   Rox_Array2D_Double output, Rox_Array2D_Float input);

//! Convert an array from uchar type to double type
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_from_float_buffer(
   Rox_Array2D_Double output, Rox_Float * input);

//! @} 

#endif
