//==============================================================================
//
//    OPENROX   : File flipud.h
//
//    Contents  : API of flipud module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FLIPUD__
#define __OPENROX_FLIPUD__

#include <generated/array2d_double.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>

//! \ingroup Array
//! \addtogroup Flip
//! @{

//! Flip a matrix using a horizontal axis (reverse the order of lines)
//! \param [out]  output   The flipped matrix
//! \param [in]   input    The matrix to flip
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_copy_flip_ud(Rox_Array2D_Double output, Rox_Array2D_Double input);

//! Flip a matrix using a horizontal axis (reverse the order of lines)
//! \param [out]  output   The flipped matrix
//! \param [in]   input    The matrix to flip
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_copy_flip_ud(Rox_Array2D_Uchar output, Rox_Array2D_Uchar input);

//! Flip an uchar matrix using a horizontal axis (reverse the order of lines)
//! \param [inout]  inout   The matrix to flip
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_flip_ud(Rox_Array2D_Uchar inout);

//! Flip an uint matrix using a horizontal axis (reverse the order of lines)
//! \param [inout]  inout   The matrix to flip
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_flip_ud(Rox_Array2D_Uint inout);

//! @} 

#endif // __OPENROX_FLIPUD__
