//============================================================================
//
//    OPENROX   : File alglt3.h
//
//    Contents  : API of lie algebra lt3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_ALGLT3__
#define __OPENROX_ALGLT3__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Algebra
//! \defgroup alglt3 alglt3
//! \brief Lie Algebra generator for lt3 group.

//! \addtogroup alglt3
//! @{

// TODO: should be a 6 rows vector to be more general
//! Build the upper triangular(3) algebra from a 5 rows vector
//! \param  [out]  alglt3         The result homogeneous 3x3 matrix
//! \param  [in ]  vector         The input 5x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_lt3generator ( Rox_Array2D_Double alglt3, Rox_Array2D_Double vector);

//! @}

#endif
