//==============================================================================
//
//    OPENROX   : File array2d_uchar_from_uint.h
//
//    Contents  : API of array2d_uchar_from_uint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_UCHAR_FROM_UINT__
#define __OPENROX_UCHAR_FROM_UINT__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>

//! \ingroup  Data_Conversion
//! \defgroup Uint2Uchar Uint2Uchar
//! \brief Conversion from Uchar to Uint.

//! \addtogroup Uint2Uchar
//! @{

//! Convert an array from uint type to uchar type
//! \param  [in]  dest              the converted array
//! \param  [in]  source            the array to convert
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_from_uint_mask(Rox_Array2D_Uchar dest, Rox_Array2D_Uint source);

//! @}

#endif // __OPENROX_UINT_TO_UCHAR__
