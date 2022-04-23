//==============================================================================
//
//    OPENROX   : File bnot.h
//
//    Contents  : API of bnot module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BNOT__
#define __OPENROX_BNOT__

#include <generated/array2d_uint.h>

//! \ingroup Array2D_Uint
//! \addtogroup BinaryNot
//! @{

//! Bitwise not of a uint array
//! \param  [out]  dest           The result array
//! \param  [in ]  source         The input array
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_bnot (
   Rox_Array2D_Uint dest, 
   const Rox_Array2D_Uint source);

//! @} 

#endif

