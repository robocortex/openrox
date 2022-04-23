//==============================================================================
//
//    OPENROX   : File detgl3.h
//
//    Contents  : API of detgl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DETGL3__
#define __OPENROX_DETGL3__

#include <generated/array2d_double.h>

//! \ingroup Matrix
//! \addtogroup Determinant
//! @{

//! Compute the determinant of a 3*3 matrix
//! \param determinant the output determinant
//! \param input the input matrix
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_detgl3(Rox_Double * determinant, Rox_Array2D_Double input);

//! @}

#endif // __OPENROX_DETGL3__
