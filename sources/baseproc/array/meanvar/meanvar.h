//==============================================================================
//
//    OPENROX   : File meanvar.h
//
//    Contents  : API of meanvar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MEANVAR__
#define __OPENROX_MEANVAR__

#include <generated/array2d_double.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>

//! \ingroup Statistics
//! \addtogroup Meanvar
//!   @{

//! Compute the mean and variance of an array (A mask is used to ignore some cells where mask is 0)
//! \param [out] mean the computed mean
//! \param [out] variance the computed variance
//! \param [in] input the input array
//! \param [in] mask the input mask
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_float_meanvar(Rox_Double *mean, Rox_Double *variance, Rox_Array2D_Float input, Rox_Array2D_Uint mask);

//! Compute the mean and variance of an array (A mask is used to ignore some cells where mask is 0)
//! \param [out] mean the computed mean
//! \param [out] variance the computed variance
//! \param [in] input the input array
//! \param [in] mask the input mask
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_meanvar(Rox_Double *mean, Rox_Double *variance, Rox_Array2D_Double input, Rox_Array2D_Uint mask);

//! @} 

#endif // __OPENROX_MEANVAR__
