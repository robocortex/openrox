//============================================================================
//
//    OPENROX   : File real_eigenvalues_eigenvectors.h
//
//    Contents  : API of matso3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REAL_EIGENVALUES_EIGENVECTORS__
#define __OPENROX_REAL_EIGENVALUES_EIGENVECTORS__

#include <generated/objset_array2d_double.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_double.h>

//! \ingroup Eigenv
//! \addtogroup Eigenv
//! @{


//! Compute eseigenvalues and eigenvectors of a matrix 
//! \param  [out]  V              The computed eigenvectors 
//! \param  [out]  e              The eigenvalues of the matrix
//! \param  [in ]  M              The matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_real_eigenvalues_eigenvectors(Rox_DynVec_Double e, Rox_ObjSet_Array2D_Double V, const Rox_Array2D_Double M);

//! @}

#endif // __OPENROX_REAL_EIGENVALUES_EIGENVECTORS__
