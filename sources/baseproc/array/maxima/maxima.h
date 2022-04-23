//==============================================================================
//
//    OPENROX   : File maxima.h
//
//  	Contents  : API of maxima module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MAXIMA__
#define __OPENROX_MAXIMA__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/dynvec_double.h>
#include <generated/dynvec_float.h>
#include <generated/dynvec_point2d_uint.h>

//! \ingroup Basemaths
//! \addtogroup maxima
//! @{

//! Compute the min and max of an array
//! \param [out] min the computed min
//! \param [out] max the computed max
//! \param [in] input the input array
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_double_local_maxima(Rox_DynVec_Double local_maxima_values, Rox_DynVec_Double local_maxima_coords, Rox_DynVec_Point2D_Uint local_maxima_indexes, Rox_Array2D_Double yv, Rox_Double y_thresh);

//! Compute the min and max of an array
//! \param [out] min the computed min
//! \param [out] max the computed max
//! \param [in] input the input array
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_array2d_float_local_maxima(Rox_DynVec_Float local_maxima_values, Rox_DynVec_Float local_maxima_coords, Rox_DynVec_Point2D_Uint local_maxima_indexes, Rox_Array2D_Float yv, Rox_Float y_thresh);

//! @}

#endif // __OPENROX_MAXIMA__
