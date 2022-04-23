//==============================================================================
//
//    OPENROX   : File detgl2.h
//
//    Contents  : API of detgl2 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DETGL2__
#define __OPENROX_DETGL2__

//! \ingroup Matrix
//! \addtogroup Determinant
//! @{

#include <generated/array2d_double.h>

//! Compute the determinant of a 2*2 matrix
//! \param determinant the output determinant
//! \param input the input matrix
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_detgl2(Rox_Double * determinant, Rox_Array2D_Double input);

//! @}

#endif // __OPENROX_DETGL2__
