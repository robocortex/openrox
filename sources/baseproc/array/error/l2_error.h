//==============================================================================
//
//    OPENROX   : File l2_error.h
//
//    Contents  : API of l2_error module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_L2_ERROR__
#define __OPENROX_L2_ERROR__

#include <generated/array2d_uint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \ingroup Vector
//! \addtogroup arrayError
//! @{

//! For each element compute A-B-mean(A-B)
//! \param  [out]  error          A pointer to the result L2 error for the whole image
//! \param  [in ]  error_img      The array with errors per pixel
//! \param  [in ]  mask           The input mask
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_l2_error (
   Rox_Double * error, 
   Rox_Array2D_Float error_img, 
   Rox_Array2D_Uint mask
);

ROX_API Rox_ErrorCode rox_array2d_double_difference_l2_norm (
   Rox_Double * l2_error, 
   const Rox_Array2D_Double array2d_1, 
   const Rox_Array2D_Double array2d_2
);

ROX_API Rox_ErrorCode rox_array2d_float_difference_l2_norm (
   Rox_Float * l2_error, 
   const Rox_Array2D_Float array2d_1, 
   const Rox_Array2D_Float array2d_2
);

ROX_API Rox_ErrorCode rox_array2d_uchar_difference_l2_norm (
   Rox_Float * l2_error, 
   const Rox_Array2D_Uchar array2d_1, 
   const Rox_Array2D_Uchar array2d_2
);

ROX_API Rox_ErrorCode rox_array2d_double_difference_l2_norm_mask (
   Rox_Double * l2_error, 
   const Rox_Array2D_Double array2d_1, 
   const Rox_Array2D_Double array2d_2,
   const Rox_Array2D_Uint mask
);

ROX_API Rox_ErrorCode rox_array2d_float_difference_l2_norm_mask (
   Rox_Float * l2_error, 
   const Rox_Array2D_Float array2d_1, 
   const Rox_Array2D_Float array2d_2,
   const Rox_Array2D_Uint mask
);

ROX_API Rox_ErrorCode rox_array2d_uchar_difference_l2_norm_mask (
   Rox_Float * l2_error, 
   const Rox_Array2D_Uchar array2d_1, 
   const Rox_Array2D_Uchar array2d_2,
   const Rox_Array2D_Uint mask
);

//! @} 

#endif // __OPENROX_L2_ERROR__
