//==============================================================================
//
//    OPENROX   : File minmax.h
//
//  	Contents  : API of minmax module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MINMAX__
#define __OPENROX_MINMAX__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

//! \ingroup Basemaths
//! \addtogroup MinMax
//! @{

//! Compute the min and max of an array
//! \param [out] min the computed min
//! \param [out] max the computed max
//! \param [in] input the input array
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_minmax(Rox_Double * min, Rox_Double * max, Rox_Array2D_Double input);

//! Compute the min and max of an array
//! \param [out] min the computed min
//! \param [out] max the computed max
//! \param [in] input the input array
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_float_minmax(Rox_Float * min, Rox_Float * max, Rox_Array2D_Float input);

//! @}

#endif
