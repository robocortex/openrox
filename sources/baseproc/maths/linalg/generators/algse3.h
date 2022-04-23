//==============================================================================
//
//    OPENROX   : File algse3.h
//
//    Contents  : API of algse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ALGSE3__
#define __OPENROX_ALGSE3__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \defgroup Lie_Algebra Lie Algebra
//! \brief Lie Algebra : vector space together with a non-associative multiplication called "Lie bracket"

//! \ingroup  Lie_Algebra
//! \defgroup se3generator se3generator
//! \brief Lie Algebra generator for SE3 group.

//! \addtogroup se3generator
//! @{

//! Build the se3 algebra from a 6 rows vector (3 translations first then 3 rotation)
//! \param  [out]  algse3         The result homogeneous 4x4 matrix
//! \param  [in ]  vector         The input 6x1 vector v = [vt; vr] (vt : translation velocity; vr : rotation velocity)
//! \return An error code
//! \todo   To be tested, rename rox_algse3_set_velocity to rox_algse3_set_velocity
ROX_API Rox_ErrorCode rox_algse3_set_velocity ( Rox_Array2D_Double algse3, Rox_Array2D_Double vector );

ROX_API Rox_ErrorCode rox_algse3_get_velocity ( Rox_Array2D_Double vector, Rox_Array2D_Double algse3 );

//! @}

#endif
