//==============================================================================
//
//    OPENROX   : File nonmax.h
//
//    Contents  : API of nonmax module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_NONMAX__
#define __OPENROX_NONMAX__

#include <generated/array2d_float.h>

//! \ingroup Image
//! \addtogroup NonMax
//! @{

//! Non Maximal suppression (set -FLT_MAX to values which are not the maximal values in their neighboorhood)
//! \param [in] output the suppressed image
//! \param [in] input the original array
//! \param [in] radius the radius of search for each piel
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_float_nonmax(Rox_Array2D_Float output, Rox_Array2D_Float input, Rox_Uint radius);

//! @} 

#endif
