//==============================================================================
//
//    OPENROX   : File integralsum.h
//
//    Contents  : API of integralsum module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTEGRALSUM__
#define __OPENROX_INTEGRALSUM__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/array2d_slint.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_sint.h>
#include <generated/array2d_uchar.h>

//! \ingroup Image
//! \addtogroup integral
//! @{

//! Compute the sum integral image of a 2D array
//! \param  [out]                 intsum the sum integral
//! \param  [in ]                 input the input array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_sint_integralsum ( Rox_Array2D_Slint intsum, Rox_Array2D_Sint input );

//! Compute the sum integral image of a 2D array
//! \param  [out]  intsum         The sum integral
//! \param  [in ]  input          The input array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_integralsum ( Rox_Array2D_Double intsum, Rox_Array2D_Double input);

//! Compute the sum integral image of a 2D array
//! \param  [out]  intsum         the sum integral
//! \param  [in ]  input          the input array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_integralsum ( Rox_Array2D_Float intsum, Rox_Array2D_Float input );

//! Compute the sum integral image of a 2D array
//! \param  [out]  intsum         the sum integral
//! \param  [in ]  input          the input array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_integralsum ( Rox_Array2D_Uint intsum, Rox_Array2D_Uchar input );

//! @} 

#endif // __OPENROX_INTEGRALSUM__
