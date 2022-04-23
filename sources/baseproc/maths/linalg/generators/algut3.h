//============================================================================
//
//    OPENROX   : File algut3.h
//
//    Contents  : API of lie algebra ut3 generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_ALGUT3__
#define __OPENROX_ALGUT3__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Algebra
//! \defgroup ut3generator ut3generator
//! \brief Lie Algebra generator for ut3 group.

//! \addtogroup ut3generator
//! @{

// TODO: should be a 6 rows vector to be more general
//! Build the upper triangular(3) algebra from a 5 rows vector
//! \param  [out]  algut3         The result homogeneous 3x3 matrix
//! \param  [in ]  vector         The input 5x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_ut3generator ( Rox_Array2D_Double algut3, Rox_Array2D_Double vector);

//! @}

#endif // __OPENROX_ALGUT3__
