//==============================================================================
//
//    OPENROX   : File ssd.h
//
//    Contents  : API of ssd module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SSD__
#define __OPENROX_SSD__

#include <generated/array2d_uint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \ingroup Vector
//! \addtogroup arrayError
//! @{

//! For each element compute sum of squares
//! \param  [out]  error          A pointer to the result L2 error 
//! \param  [in ]  error_img      The array with errors per pixel
//! \param  [in ]  mask           The input mask
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_ss_mask (
   Rox_Double * ss, 
   Rox_Array2D_Float array2d, 
   Rox_Array2D_Uint mask
);

ROX_API Rox_ErrorCode rox_array2d_double_ssd (
   Rox_Double * ssd, 
   const Rox_Array2D_Double array2d_1, 
   const Rox_Array2D_Double array2d_2
);

ROX_API Rox_ErrorCode rox_array2d_float_ssd (
   Rox_Float * ssd, 
   const Rox_Array2D_Float array2d_1, 
   const Rox_Array2D_Float array2d_2
);

ROX_API Rox_ErrorCode rox_array2d_uchar_ssd (
   Rox_Float * ssd, 
   const Rox_Array2D_Uchar array2d_1, 
   const Rox_Array2D_Uchar array2d_2
);

ROX_API Rox_ErrorCode rox_array2d_double_ssd_mask (
   Rox_Double * ssd, 
   const Rox_Array2D_Double array2d_1, 
   const Rox_Array2D_Double array2d_2,
   const Rox_Array2D_Uint mask
);

ROX_API Rox_ErrorCode rox_array2d_float_ssd_mask (
   Rox_Float * ssd, 
   const Rox_Array2D_Float array2d_1, 
   const Rox_Array2D_Float array2d_2,
   const Rox_Array2D_Uint mask
);

ROX_API Rox_ErrorCode rox_array2d_uchar_ssd_mask (
   Rox_Float * ssd, 
   const Rox_Array2D_Uchar array2d_1, 
   const Rox_Array2D_Uchar array2d_2,
   const Rox_Array2D_Uint mask
);

//! @} 

#endif
