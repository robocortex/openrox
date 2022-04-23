//==============================================================================
//
//    OPENROX   : File gausspivoting.h
//
//    Contents  : API of gausspivoting module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_GAUSS_PIVOTING__
#define __OPENROX_GAUSS_PIVOTING__

#include <generated/array2d_double.h>

//! \ingroup  Linalg
//! \defgroup Gauss_Pivoting Gauss Pivoting
//! \brief Gauss Pivoting Decomposition.

//! \addtogroup Gauss_Pivoting
//! @{

//! Compute the gauss-jordan row echelon form of a matrix
//! \param  [out]  output 	The row echelon form matrix
//! \param  [in ]  input		The input matrix (square)
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_gauss_pivoting (
   Rox_Array2D_Double output, 
   Rox_Array2D_Double input
);

//! @} 

#endif // __OPENROX_GAUSS_PIVOTING__
