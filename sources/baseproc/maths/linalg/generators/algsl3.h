//============================================================================
//
//    OPENROX   : File sl3generator.h
//
//    Contents  : API of sl3generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_SL3GENERATOR__
#define __OPENROX_SL3GENERATOR__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Algebra
//! \defgroup sl3generator sl3generator
//! \brief Lie Algebra generator for SL3 group.

//! \addtogroup sl3generator
//! @{

//! Build the SL(3) algebra from a 8 rows vector
//! \param  [out]  algsl3         The result homogeneous 3x3 matrix
//! \param  [in ]  vector         The input 8x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_sl3generator(Rox_Array2D_Double algsl3, Rox_Array2D_Double vector);

//! @} 

#endif
