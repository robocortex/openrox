//============================================================================
//
//    OPENROX   : File set_border.h
//
//    Contents  : API of set_border module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_MASK_SET_BORDER__
#define __OPENROX_MASK_SET_BORDER__

#include <generated/array2d_uint.h>

//! \ingroup  Mask
//! \addtogroup MaskSetBorder
//! @{

//! Set the border of a imask to zero
//! \param  [out]  dest           The mask to set
//! \param  [in ]  size           The border size in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_set_border ( Rox_Array2D_Uint dest, Rox_Sint size );

//! @} 

#endif // __OPENROX_MASK_SETBORDER__
