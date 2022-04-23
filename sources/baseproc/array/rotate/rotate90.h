//==============================================================================
//
//    OPENROX   : File rotate90.h
//
//    Contents  : API of rotate90 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ROTATE90__
#define __OPENROX_ROTATE90__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>

//! \ingroup Image
//! \addtogroup Rotate90
//! @{

//! Matrix rotation of 90 degrees counter-clockwise
//! \param [out] res   The rotated matrix (rows = input.cols, cols = input.rows)
//! \param [in]  input The input array
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_rotate90(Rox_Array2D_Uchar res, Rox_Array2D_Uchar input);

//! Matrix rotation of 90 degrees counter-clockwise
//! \param [out] res   The rotated matrix (rows = input.cols, cols = input.rows)
//! \param [in]  input The input array
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_rotate90(Rox_Array2D_Uint res, Rox_Array2D_Uint input);

//! @} 

#endif
