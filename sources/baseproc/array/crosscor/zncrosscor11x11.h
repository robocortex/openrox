//==============================================================================
//
//    OPENROX   : File zncrosscor11x11.h
//
//    Contents  : API of zncrosscor11x11 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ZNCROSSCOR11X11__
#define __OPENROX_ZNCROSSCOR11X11__

#include <generated/array2d_uchar.h>

//! \addtogroup ZNCC
//! @{

//! Compute ZNCC on a 11x11 patch.
//! \param  [out]  score         computed score
//! \param  [in ]  model         reference patch
//! \param  [in ]  ref_sum       reference patch sum
//! \param  [in ]  ref_sumsq     reference patch sum^2
//! \param  [in ]  source        current image
//! \param  [in ]  cj            u position of patch center in source image
//! \param  [in ]  ci             v position of patch center in source image
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_zncc_11x11 ( 
   Rox_Double * score, 
   const Rox_Array2D_Uchar model, 
   const Rox_Slint ref_sum, 
   const Rox_Slint ref_sumsq, 
   const Rox_Array2D_Uchar source, 
   const Rox_Sint cj, 
   const Rox_Sint ci
);

//! @} 

#endif // __OPENROX_ZNCROSSCOR11X11__
