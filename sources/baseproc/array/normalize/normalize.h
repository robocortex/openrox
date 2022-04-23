//==============================================================================
//
//    OPENROX   : File normalize.h
//
//    Contents  : API of vector normalization module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VECTOR_NORMALIZE__
#define __OPENROX_VECTOR_NORMALIZE__

#include <generated/array2d_double.h>

//! \ingroup Linalg
//! \defgroup Vector Vector

//! \ingroup Vector
//! \addtogroup Normalize
//! @{

ROX_API Rox_ErrorCode rox_array2d_double_normalize(Rox_Array2D_Double res, Rox_Array2D_Double inp);

//! @} 

#endif
