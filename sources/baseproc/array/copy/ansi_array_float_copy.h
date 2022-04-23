//==============================================================================
//
//    OPENROX   : File ansi_array_float_copy.h
//
//  	Contents  : API of array_float_copy module in ANSI C99
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_ANSI_ARRAY_FLOAT_COPY__
#define __OPENROX_ANSI_ARRAY_FLOAT_COPY__

#include <generated/array2d_float.h>

//! \addtogroup Array
//! @{

//! Convert an array from uchar type to double type
//! \param  [out] output            the converted array
//! \param  [in ]  input            the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API int rox_ansi_array_float_copy ( float * out, const float * inp, const int size );

//! @} 

#endif // __OPENROX_ANSI_ARRAY_FLOAT_COPY__
