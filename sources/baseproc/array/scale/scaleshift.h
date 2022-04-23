//==============================================================================
//
//    OPENROX   : File scaleshift.h
//
//    Contents  : API of scaleshift module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SCALESHIFT__
#define __OPENROX_SCALESHIFT__

#include <generated/array2d_float.h>

//! \ingroup Matrix
//! \addtogroup ScaleShift
//! @{
 
//! Scale then Shift all elements of an array
//! \param [out]  res      The destination array
//! \param [in]   one      The source array
//! \param [in]   scale    The scale to apply (a scalar)
//! \param [in]   shift    The shift to apply (a scalar)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_float_scaleshift(Rox_Array2D_Float res, Rox_Array2D_Float one, Rox_Float scale, Rox_Float shift);

//! @} 

#endif // __OPENROX_SCALESHIFT__
