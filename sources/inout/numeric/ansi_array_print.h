//==============================================================================
//
//    OPENROX   : File ansi array_print.h
//
//    Contents  : API of ansi array print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_ARRAY_PRINT__
#define __OPENROX_ANSI_ARRAY_PRINT__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>
#include <stdlib.h>

//! \addtogroup Array
//! @{

//! Print an array on stdout
//! \param  [in ]  input the array to print
//! \return An error code
ROX_API int rox_ansi_array_double_print ( double * input, size_t size );

//! Print an array on stdout
//! \param  [in ]  input the array to print
//! \return An error code
ROX_API int rox_ansi_array_float_print ( float * input, size_t size );

//! Print an array on stdout
//! \param  [in ]  input the array to print
//! \return An error code
ROX_API int rox_ansi_array_uint_print ( unsigned int * input, size_t size );

//! Print an array on stdout with a given precsion
//! \param  [in ]  input the array to print
//! \param  [in ]  precision number of digits after the dot
//! \return An error code
ROX_API int rox_ansi_array_double_print_precision ( double * input, size_t size, int precision );

//! Print an array of size rows x cols on stdout in matrix form 
//! \param  [in ]  input 			 The array to print
//! \param  [in ]  rows 			 The rows
//! \param  [in ]  cols 			 The cols
//! \return An error code
ROX_API int rox_ansi_array_float_print_as_array2d ( float * input, size_t rows, size_t cols );

ROX_API int rox_ansi_array_double_print_as_array2d ( double * input, size_t rows, size_t cols );

//! @} 

#endif
