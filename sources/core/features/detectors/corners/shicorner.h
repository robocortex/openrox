//==============================================================================
//
//    OPENROX   : File shicorner.h
//
//    Contents  : API of shicorner module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SHICORNER__
#define __OPENROX_SHICORNER__

#include <generated/array2d_uchar.h>

//! \ingroup Detectors
//! \addtogroup SHI-TOMASI
//! @{

//! Shi-Tomasi corner test (minima of eigenvalues of covariance matrix of gradients).
//! \remark No bounds checking is done for speed sake.
//! \param  [out] response the output corner response for this pixel
//! \param  [in] source the raw image data pointer.
//! \param  [in] i the row of interest
//! \param  [in] j the column of interest
//! \param  [in] radius the image region radius used
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_shicorner_test (
   Rox_Float *response, 
   Rox_Uchar ** source, 
   Rox_Uint i, 
   Rox_Uint j, 
   Rox_Uint radius
);

//! @}

#endif
