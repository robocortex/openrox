//==============================================================================
//
//    OPENROX   : File symmetrise.h
//
//    Contents  : API of symmetrise module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SYMMETRISE__
#define __OPENROX_SYMMETRISE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup symmetrise
//! @{

//! Symmetrise an array2d by copying the lower triangular part into the upper triangular part   
//! \warning This function is valid only for square array2d
//! \param  [out]  array2d_lower  The symmetrised matrix (L[v][u] = L[u][v])
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_symmetrise_lower ( Rox_Array2D_Double array2d_lower );

//! Symmetrise an array2d by copying the upper triangular part into the lower triangular part   
//! \warning This function is valid only for square array2d
//! \param  [out]  array2d_upper  The symmetrised matrix (L[v][u] = L[u][v])
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_symmetrise_upper ( Rox_Array2D_Double array2d_upper );

//! @} 

#endif
