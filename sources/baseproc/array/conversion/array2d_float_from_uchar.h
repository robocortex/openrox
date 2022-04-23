//==============================================================================
//
//    OPENROX   : File array2d_float_from_uchar.h
//
//  	Contents  : API of array2d_uchar_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_FLOAT_FROM_UCHAR__
#define __OPENROX_FLOAT_FROM_UCHAR__

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>

//! \ingroup Array2D
//! \defgroup Data_Conversion Data Conversion
//! \brief Types (re)definititon.

//! \ingroup  Data_Conversion
//! \defgroup Uchar2Float Uchar2Float
//! \brief Conversion from Uchar to Float.

//! \addtogroup Uchar2Float
//! @{

//! Convert an array from uchar type to float type
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_from_uchar(Rox_Array2D_Float output, const Rox_Array2D_Uchar input);

//! Convert an array from uchar type to float type and scale values to [0;1]
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_from_uchar_normalize(Rox_Array2D_Float output, const Rox_Array2D_Uchar input);

//! Convert an array from uchar type to float type and scale values to [0;1] using min max values
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_from_uchar_normalize_minmax(Rox_Array2D_Float output, const Rox_Array2D_Uchar input);

//! Convert an array from uchar type to a new float type and scale values to [0;1]
//! \param  [out] output            the converted array
//! \param  [in]  input             the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_new_from_uchar_normalize(Rox_Array2D_Float * output, const Rox_Array2D_Uchar input);

ROX_API Rox_ErrorCode rox_array2d_float_new_from_uchar ( Rox_Array2D_Float * output, const Rox_Array2D_Uchar input);

// This function should be in file array_float_from_uchar.h
ROX_API int rox_ansi_array_float_from_uchar ( float * output, const unsigned char * input, const int size );

//! @} 

#endif
