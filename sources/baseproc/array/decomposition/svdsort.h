//==============================================================================
//
//    OPENROX   : File svdsort.h
//
//    Contents  : API of svdsort module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SVDSORT__
#define __OPENROX_SVDSORT__

#include <generated/array2d_double.h>

//! \addtogroup SVD
//! @{

//! Given a triplet U,S,V computed by svd, rearrange matrices so that S is sorted by decreasing singular values
//! \param  [out] U     the left special result matrix
//! \param  [out] S     the middle special result vector
//! \param  [out] V     the right special result matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_svd_sort(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V);

//! Given a pair S, V computed by svd, rearrange matrices so that S is sorted by decreasing singular values
//! \param  [out] S     the middle special result vector
//! \param  [out] V     the right special result matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_svd_sort_SV(Rox_Array2D_Double S, Rox_Array2D_Double V);

//! @} 

#endif
