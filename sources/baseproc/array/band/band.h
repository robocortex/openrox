//============================================================================
//
//    OPENROX   : File band.h
//
//    Contents  : API of band module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_BAND__
#define __OPENROX_BAND__

#include <generated/array2d_uint.h>

//! \ingroup Array2D_Uint
//! \addtogroup BinaryAnd
//! @{

//! Bitwise and between two arrays and place result in a third array
//! \param  [out]  res            the result
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_band ( 
   Rox_Array2D_Uint res, 
   const Rox_Array2D_Uint one, 
   const Rox_Array2D_Uint two
);

//! @}

#endif
