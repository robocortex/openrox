//============================================================================
//
//    OPENROX   : File basegradient.h
//
//    Contents  : API of basegradient module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_BASEGRADIENT__
#define __OPENROX_BASEGRADIENT__

#include <generated/array2d_float.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Image
//! \addtogroup Gradient
//! @{

//! Compute gradient using symmetric [-1, 0, 1]/2 kernel for u and [-1; 0; 1]/2 for v
//! \warning The border are not considered 
//! \param  [out]  gradient_u        The result horizontal gradient
//! \param  [out]  gradient_v        The result vertical gradient
//! \param  [out]  imask_out         The result mask gradient
//! \param  [in ]  image             The source image
//! \param  [in ]  imask_inp         The source mask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_basegradient ( 
   Rox_Array2D_Float gradient_u, 
   Rox_Array2D_Float gradient_v, 
   Rox_Imask imask_out, 
   const Rox_Array2D_Float image, 
   const Rox_Imask imask_inp
);

//! Compute gradient using symmetric [-1, 0, 1]/2 kernel for u and [-1; 0; 1]/2 for v
//! \warning The border are not considered 
//! \param  [out]  gradient_u        The result horizontal gradient
//! \param  [out]  gradient_v        The result vertical gradient
//! \param  [in ]  image             The source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_basegradient_nomask (
   Rox_Array2D_Float gradient_u, 
   Rox_Array2D_Float gradient_v, 
   const Rox_Array2D_Float image
);

//! \todo To be added and tested
// ROX_API Rox_ErrorCode rox_array2d_float_basegradient (Rox_Array2D_Sint gradient_u, Rox_Array2D_Sint gradient_v, Rox_Array2D_Uint omask, Rox_Image image, Rox_Array2D_Uint imask);

//! @} 

#endif
