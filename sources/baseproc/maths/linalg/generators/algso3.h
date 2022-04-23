//============================================================================
//
//    OPENROX   : File so3generator.h
//
//    Contents  : API of so3generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_SO3GENERATOR__
#define __OPENROX_SO3GENERATOR__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Algebra
//! \defgroup so3generator so3generator
//! \brief Lie Algebra generator for SO3 group.

//! \addtogroup so3generator
//! @{

//! Build the so(3) algebra from a 3 rows vector
//! \param  [out]  algso3         The result skew symmetric 3x3 matrix in so(3)
//! \param  [in ]  vector         The input 3x1 vector
//! \return An error code
//! \todo   To be tested, rename the function rox_algso3_generator
ROX_API Rox_ErrorCode rox_linalg_so3generator(Rox_Array2D_Double algso3, Rox_Array2D_Double vector);

//! @}

#endif
