//==============================================================================
//
//    OPENROX   : File version.h
//
//    Contents  : API of version module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_AVX__
#define __OPENROX_AVX__

#include <stdbool.h>
#include <system/memory/datatypes.h>
#ifdef WIN32
   #include <intrin.h>
#endif

//! \defgroup SSE
//! \brief SSE tools.

//! \addtogroup SSE
//! @{

//! Get the sum of the 8 floats in the 256 bit vector
//! \param  [in] v          The 256 bit vector (8 floats = 8 x 32 bits)
//! \return The sum
//! \todo   To be tested
ROX_API float rox_mm256_hsum_ps ( __m256 var );

ROX_API void rox_mm256_printf_float ( __m256 var );

ROX_API void rox_mm256i_printf_uint16 ( __m256i var );

//! Get the absolute values of the 8 floats in the 256 bit vector
//! \param  [in] var          The 256 bit vector (8 floats = 8 x 32 bits)
//! \return The absolute values
//! \todo   To be tested
ROX_API __m256 rox_mm256_abs_ps(__m256 var);

//! Compare var to min. If one of the var float is lower than its corresponding
//! min float, the function return true.
//! \param  [in] v          The 256 bit vector (8 floats = 8 x 32 bits)
//! \return The sum
//! \todo   To be tested
ROX_API bool rox_mm256_cmplt_or(__m256 var, __m256 min);


//! @}

#endif // __OPENROX_AVX__
