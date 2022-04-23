//==============================================================================
//
//    OPENROX   : File svdinverse.h
//
//    Contents  : API of svdinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SVD_INVERSE__
#define __OPENROX_SVD_INVERSE__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Compute the pseudo inverse of a general matrix
//! \param  [out]  dest          The inversed matrix (m*n)
//! \param  [in ]  source        The general inverse (n*m)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_double_svdinverse(Rox_Array2D_Double dest, Rox_Array2D_Double source);

//! @}

#endif // __OPENROX_SVD_INVERSE__
