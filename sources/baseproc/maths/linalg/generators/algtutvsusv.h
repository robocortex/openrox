//============================================================================
//
//    OPENROX   : File tutvsusvgenerator.h
//
//    Contents  : API of tutvsusvgenerator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_TUTVSUSVGENERATOR__
#define __OPENROX_TUTVSUSVGENERATOR__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Algebra
//! \defgroup tutvsusvgenerator tutvsusvgenerator
//! \brief Lie Algebra generator for tutvsusv group.

//! \addtogroup tutvsusvgenerator
//! @{

//! Update the matrix  to the right given a 4 rows vector
//! \param  [out]  group          The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 4x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_tutvsusvupdate_right(Rox_Array2D_Double group, Rox_Array2D_Double vector);

//! Update the matrix to the left given a 8 rows vector
//! \param  [out]  group          The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 4x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_tutvsusvupdate_left(Rox_Array2D_Double group, Rox_Array2D_Double vector);

//! @} 

#endif
