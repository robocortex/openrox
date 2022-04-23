//============================================================================
//
//    OPENROX   : File set_data.h
//
//    Contents  : API of set_data module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_MASK_SETDATA__
#define __OPENROX_MASK_SETDATA__

#include <generated/array2d_uint.h>

//! \ingroup  Mask
//! \addtogroup MaskSetData
//! @{

//! Set the mask from an external buffer
//! \param  [out]  dest           The mask to set
//! \param  [in ]  source         The mask buffer
//! \param  [in ]  bytesPerRow    The octet size of one mask row
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_set_data(Rox_Array2D_Uint dest, Rox_Uint * source, Rox_Sint bytesPerRow);

//! @}   

#endif // __OPENROX_MASK_SETDATA__
