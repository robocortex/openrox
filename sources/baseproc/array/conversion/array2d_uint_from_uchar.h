//============================================================================
//
//    OPENROX   : File array2d_uint_from_uchar.h
//
//  	Contents  : API of array2d_uint_from_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_UINT_FROM_UCHAR__
#define __OPENROX_UINT_FROM_UCHAR__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>

//! \ingroup  Data_Conversion
//! \defgroup Uchar2Uint Uchar2Uint
//! \brief Conversion from Uchar to Uint.

//! \addtogroup Uchar2Uint
//! @{

//! Convert an array from uchar type to uint type
//! \param  [in]  output	           The converted array
//! \param  [in]  input		           The array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_from_uchar_mask(Rox_Array2D_Uint output, Rox_Array2D_Uchar input);

//! @} 

#endif // __OPENROX_UCHAR_TO_UINT__
