//==============================================================================
//
//    OPENROX   : File sse.h
//
//    Contents  : API of sse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SSE__
#define __OPENROX_SSE__

#include <stdbool.h>
#include <system/memory/datatypes.h>

//! \defgroup SSE
//! \brief SSE tools.

//! \addtogroup SSE
//! @{

//! Get the sum of the 4 floats in the 128 bit vector
//! \param  [in] v          The 128 bit vector (4 floats = 4 x 32 bits)
//! \return The sum of the 4 floats in the 128 bit vector
//! \todo   To be tested
ROX_API float rox_mm128_hsum_ps ( __m128 var );

ROX_API void rox_mm128_printf_float ( __m128 var );

ROX_API void rox_mm128i_printf_uint16 ( __m128i var );

ROX_API void rox_mm128i_printf_uint8 ( __m128i var );

//! Get the absolute values of the 4 floats in the 128 bit vector
//! \param  [in] var          The 256 bit vector (8 floats = 8 x 32 bits)
//! \return The absolute values
//! \todo   To be tested
ROX_API __m128 rox_mm128_abs_ps(__m128 var);

//! Compare var to min. If one of the var float is lower than its corresponding
//! min float, the function return true.
//! \param  [in] v          The 256 bit vector (8 floats = 8 x 32 bits)
//! \return The sum
//! \todo   To be tested
ROX_API bool rox_mm128_cmplt_or(__m128 var, __m128 min);

ROX_API void rox_mm128i_loadu_16_uchar_to_vectors_8_short ( __m128i * vector_8_short_1, __m128i * vector_8_short_2, unsigned char * data_16_uchar );

//! @}

#endif // __OPENROX_SSE__