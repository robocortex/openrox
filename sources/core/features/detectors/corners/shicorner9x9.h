//==============================================================================
//
//    OPENROX   : File shicorner9x9.h
//
//    Contents  : API of shicorner9x9 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SHICORNER9X9__
#define __OPENROX_SHICORNER9X9__

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

//! \ingroup Detectors
//! \addtogroup SHI-TOMASI
//! @{

//! Shi-Tomasi corner test (minima of eigenvalues of covariance matrix of gradients).
//! \param  [out]  response       The output corner response for this pixel
//! \param  [in ]  source         The raw image data pointer.
//! \param  [in ]  v              The row of interest
//! \param  [in ]  u              The column of interest
//! \return An error code
//! \remark No bounds checking is done for speed sake.
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_shicorner9x9_test (
   Rox_Float * response, 
   Rox_Uchar ** source, 
   const Rox_Uint v, 
   const Rox_Uint u
);

//! @} 

#endif
