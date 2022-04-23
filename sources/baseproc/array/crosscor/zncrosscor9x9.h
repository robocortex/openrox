//==============================================================================
//
//    OPENROX   : File zncrosscor9x9.h
//
//    Contents  : API of zncrosscor9x9 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ZNCROSSCOR9X9__
#define __OPENROX_ZNCROSSCOR9X9__

#include <generated/array2d_uchar.h>

//! \addtogroup ZNCC
//! @{

//! Compute ZNCC on a 9x9 patch.
//! \param  [out]  score          Computed score
//! \param  [in ]  model          Reference patch
//! \param  [in ]  ref_sum        Reference patch sum
//! \param  [in ]  ref_sumsq      Reference patch sum^2
//! \param  [in ]  source         Current image
//! \param  [in ]  cj             The u position of patch center in source image
//! \param  [in ]  ci             The v position of patch center in source image
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_zncc_9x9 (
   Rox_Double * score, 
   const Rox_Array2D_Uchar model, 
   const Rox_Slint ref_sum, 
   const Rox_Slint ref_sumsq, 
   const Rox_Array2D_Uchar source, 
   const Rox_Sint cj, 
   const Rox_Sint ci
);

//! @}

#endif // __OPENROX_ZNCROSSCOR9X9__
