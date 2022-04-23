//==============================================================================
//
//    OPENROX   : File neon.h
//
//    Contents  : API of NEON 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_NEON__
#define __OPENROX_NEON__

#include <stdbool.h>
#include <system/memory/datatypes.h>

//! \defgroup NEON
//! \brief NEON tools.

//! \addtogroup NEON
//! @{

#if defined (__arm__)
//! Divides the floating-point values  of the two provided vectors
//! \param  [in] a          First vector
//! \param  [in] b          Second vector
//! \return The division of the 4 floats in the 128 bit vector
//! \todo   To be tested
ROX_API float32x4_t vdivq_f32(float32x4_t a, float32x4_t b);
#endif


//! Move the upper 2 single-precision (32-bit) floating-point elements from b to the lower 2 elements of dst, 
//! and copy the upper 2 elements from a to the upper 2 elements of dst (based on _mm_movehl_ps from SSE).
//! \param  [in] a          First vector
//! \param  [in] b          Second vector
//! \return Resulting vector
//! \todo   To be tested
ROX_API float32x4_t rox_f32_movehl_ps(float32x4_t __A, float32x4_t __B);


//! Add the lower single-precision (32-bit) floating-point element in a and b, 
//! store the result in the lower element of dst, and copy the upper 3 packed elements 
//! from a to the upper elements of dst (based on _mm_add_ss from SSE). 
//! \param  [in] var          The 256 bit vector (8 floats = 8 x 32 bits)
//! \return Resulting vector
//! \todo   To be tested
ROX_API float32x4_t rox_f32_add_ss(float32x4_t a, float32x4_t b);

//! Get the sum of the 4 floats in the 128 bit vector
//! \param  [in] v          The 128 bit vector (4 floats = 4 x 32 bits)
//! \return The sum of the 4 floats in the 128 bit vector
//! \todo   To be tested
ROX_API float rox_f32_hsum_ps(float32x4_t var);

//! Compare var to min. If one of the var float is lower than its corresponding
//! min float, the function return true.
//! \param  [in] var  Vector to be compared
//! \param  [in] min  Minimum to be compared
//! \return True if comparison is true. False ortherwise.
//! \todo   To be tested
ROX_API int rox_f32_cmplt_or(float32x4_t var, float32x4_t min);


//! @}

#endif // __OPENROX_SSE__