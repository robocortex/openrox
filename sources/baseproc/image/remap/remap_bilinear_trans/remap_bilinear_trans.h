//============================================================================
//
//    OPENROX   : File remapbilinear_trans.h
//
//    Contents  : API of remapbilinear_trans module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_REMAPBILINEAR_TRANS__
#define __OPENROX_REMAPBILINEAR_TRANS__

#include <generated/array2d_float.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out] output               the result array
//! \param  [out] output_mask_output   the result array validity mask
//! \param  [in]  input                the input array
//! \param  [in]  tx                   the x-translation
//! \param  [in]  ty                   the y-translation
//! \return An error code 
ROX_API Rox_ErrorCode rox_remap_bilinear_trans_uchar_to_float (
   Rox_Array2D_Float output, 
   Rox_Imask output_mask_output, 
   const Rox_Image input, 
   const Rox_Float tx, 
   const Rox_Float ty
);

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out]  output               the result array
//! \param  [out]  output_mask_output   the result array validity mask
//! \param  [in ]  input                the input array
//! \param  [in ]  tx                   the x-translation
//! \param  [in ]  ty                   the y-translation
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_bilinear_trans_uchar_to_uchar (
   Rox_Image output, 
   Rox_Imask output_mask_output, 
   const Rox_Image input, 
   const Rox_Float tx, 
   const Rox_Float ty
);

//! @} 

#endif
