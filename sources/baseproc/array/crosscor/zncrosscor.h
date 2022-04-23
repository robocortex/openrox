//==============================================================================
//
//    OPENROX   : File zncrosscor.h
//
//    Contents  : API of zncrosscor module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ZNCROSSCOR__
#define __OPENROX_ZNCROSSCOR__

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>

#include <baseproc/array/crosscor/array2d_uchar_zncc.h>
#include <baseproc/array/crosscor/array2d_float_zncc.h>

#include <baseproc/array/crosscor/array2d_uchar_zncc_nomask.h>
#include <baseproc/array/crosscor/array2d_float_zncc_nomask.h>

//! \ingroup Image
//! \addtogroup ZNCC
//!  @{

//! Compute the zero normalized cross correlation between two arrays (A mask is used to ignore some cells where mask is 0)
//! \param  [out]  score          the computed cross correlation with value normalized between  0 and 1 ((1 + zncc) / 2)
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \param  [in ]  mask           the input mask
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_zncc_normalizedscore ( 
   Rox_Double * score, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two, 
   const Rox_Array2D_Uint mask
);

//! Compute the zero normalized cross correlation between two arrays (A mask is used to ignore some cells where mask is 0)
//! \param  [out]  score          the computed cross correlation with value normalized between  0 and 1 ((1 + zncc) / 2)
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_zncc_nomask_normalizedscore (
   Rox_Double *score, 
   const Rox_Array2D_Float one, 
   const Rox_Array2D_Float two
);

//! Compute the zero normalized cross correlation between two arrays (A mask is used to ignore some cells where mask is 0)
//! \param  [out]  score          the computed cross correlation with value normalized between  0 and 1 ((1 + zncc) / 2)
//! \param  [in ]  one            the left operand
//! \param  [in ]  two            the right operand
//! \param  [in ]  mask           the input mask
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_zncc_normalizedscore (
   Rox_Double * score, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two, 
   const Rox_Array2D_Uint mask
);

//! Compute the zero normalized cross correlation between two arrays (A mask is used to ignore some cells where mask is 0)
//! \param  [out]  score the computed cross correlation with value normalized between  0 and 1 ((1 + zncc) / 2)
//! \param  [in ]  one the left operand
//! \param  [in ]  two the right operand
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_zncc_nomask_normalizedscore (
   Rox_Double * score, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two
);

//! @} 

#endif // __OPENROX_ZNCROOSCOR__
