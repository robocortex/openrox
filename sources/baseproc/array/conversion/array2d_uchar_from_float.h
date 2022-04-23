//==============================================================================
//
//    OPENROX   : File array2d_uchar__from_float.h
//
//    Contents  : API of array2d_uchar_from_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_UCHAR_FROM_FLOAT__
#define __OPENROX_UCHAR_FROM_FLOAT__

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>

//! \ingroup  Data_Conversion
//! \defgroup Float2Uchar Float2Uchar
//! \brief Conversion from Uchar to Float.

//! \addtogroup Float2Uchar
//! @{
 
//! Convert an array from float type to uchar type
//! \param  [in]  output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_from_float(Rox_Array2D_Uchar output, Rox_Array2D_Float input);

//! Convert an array from normalized float type to uchar type
//! \param  [in]  output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_from_float_normalize(Rox_Array2D_Uchar output, Rox_Array2D_Float input);

//! @} 

#endif
