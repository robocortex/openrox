//==============================================================================
//
//    OPENROX   : File fillzero.h
//
//    Contents  : API of fillzero module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FILLZERO__
#define __OPENROX_FILLZERO__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>

//! \ingroup Array2D_Double
//! \brief Fill array with zero everywhere 
//! \param [out] dest the 2D array
//! \return an error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_fillzero(Rox_Array2D_Double dest);

//! \ingroup Array2D_Double
//! \brief Fill array with zero everywhere 
//! \param [out] dest the 2D array
//! \return an error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_float_fillzero(Rox_Array2D_Float dest);

//! \ingroup Array2D_Double
//! \brief Fill array with zero everywhere 
//! \param [out] dest the 2D array
//! \return an error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_uint_fillzero(Rox_Array2D_Uint dest);

#endif
