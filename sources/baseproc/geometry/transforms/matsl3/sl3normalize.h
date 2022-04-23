//==============================================================================
//
//    OPENROX   : File sl3normalize.h
//
//    Contents  : API of sl3normalize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SL3_NORMALIZE__
#define __OPENROX_SL3_NORMALIZE__

#include <generated/array2d_double.h>

//! \ingroup LinAlg
//! \defgroup MatSL3 MatSL3

//! \ingroup MatSL3
//! \addtogroup Normalize
//!   @{

//! Update matrix33 scale so that its determinant equals to one (SL(3))
//! \param  [out]  homography     The MatSL3 output
//! \param  [in ]  matrix33       A generic 3*3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_normalize ( Rox_Array2D_Double homography, Rox_Array2D_Double matrix33 );

//! @} 

#endif
