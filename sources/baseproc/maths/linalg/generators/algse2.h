//============================================================================
//
//    OPENROX   : File algse2.h
//
//    Contents  : API of se2generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_ALGSE2__
#define __OPENROX_ALGSE2__

#include <generated/array2d_double.h>

//! \ingroup  Lie_Algebra
//! \defgroup se2generator se2generator
//! \brief Lie Algebra generator for SE2 group.

//! \addtogroup se2generator
//! @{

//! Build the SE(2) algebra from a 3 rows vector (2 translation then 1 rotation)
//! \param  [out]  algebra        The result homogeneous 3x3 matrix
//! \param  [in ]  vector         The input 3x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_se2generator(Rox_Array2D_Double algebra, Rox_Array2D_Double vector);

//! Update the pose to the right given a 6 rows vector (2 translation then 1 rotation))
//! \param  [out]  pose           The result AND input homogeneous 3x3 matrix
//! \param  [in ]  vector         The input 3x1 vector
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_linalg_se2update_right(Rox_Array2D_Double pose, Rox_Array2D_Double vector);

//! Update the pose to the left given a 6 rows vector (2 translation then 1 rotation))
//! \param  [out]  pose           The result AND input homogeneous 3x3 matrix
//! \param  [in ]  vector         The input 3x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linalg_se2update_left(Rox_Array2D_Double pose, Rox_Array2D_Double vector);

//! @}

#endif //
