//==============================================================================
//
//    OPENROX   : File array2d_print.h
//
//    Contents  : API of array2d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D_PRINT__
#define __OPENROX_ARRAY2D_PRINT__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \addtogroup Array2D
//! @{

//! Display a 2D array on stdout
//! \param [in] input the array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_print(Rox_Array2D_Double input);

//! Display a 2D array on stdout
//! \param [in] input the array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_print(Rox_Array2D_Float input);

//! Save a 2D array on stream
//! \param [in] input the array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_print(Rox_Array2D_Uint input);

//! Save a 2D array on stream
//! \param [in] input the array to print
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_print(Rox_Array2D_Uchar input);

//! Display a 2D array on stdout with a given precsion
//! \param [in] input the array to print
//! \param [in] precision number of digits after the dot
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_print_precision(Rox_Array2D_Double input, Rox_Sint precision);

//! @} 

#endif
