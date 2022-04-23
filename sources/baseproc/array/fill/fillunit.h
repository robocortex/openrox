//==============================================================================
//
//    OPENROX   : File fillunit.h
//
//  	Contents  : API of fillunit module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_FILLUNIT__
#define __OPENROX_FILLUNIT__

#include <generated/array2d_double.h>

//! \ingroup Array2D_Double
//! \brief Fill array with zero everywhere except on the diagonal with 1
//! \param [out] inpout the 2D array
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_double_fillunit(Rox_Array2D_Double inpout);

#endif
