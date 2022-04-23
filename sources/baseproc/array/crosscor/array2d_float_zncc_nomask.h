//==============================================================================
//
//    OPENROX   : File array2d_float_zncc_nomask.h
//
//    Contents  : API of array2d_float_zncc_nomask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D_FLOAT_ZNCC_NOMASK__
#define __OPENROX_ARRAY2D_FLOAT_ZNCC_NOMASK__

#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>

//! \ingroup Image
//! \addtogroup ZNCC
//!  @{

//! Compute the zero normalized cross correlation between two arrays (A mask is used to ignore some cells where mask is 0)
//! \param  [out]  zncc           the computed cross correlation
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_zncc_nomask (
   Rox_Double *zncc, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two
);

//! @} 

#endif // __OPENROX_ARRAY2D_FLOAT_ZNCC_NOMASK__
