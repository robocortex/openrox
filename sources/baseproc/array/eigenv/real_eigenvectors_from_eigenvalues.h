//============================================================================
//
//    OPENROX   : File real_eigenvectors_from_eigenvalues.h
//
//    Contents  : API of real_eigenvectors_from_eigenvalues module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REAL_EIGENVECTORS_FROM_EIGENVALUES__
#define __OPENROX_REAL_EIGENVECTORS_FROM_EIGENVALUES__

#include <generated/objset_array2d_double.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_double.h>

//! \ingroup Eigenv
//! \addtogroup Eigenv
//! @{

//! Compute eigenvectors of a matrix knowing its eseigenvalues
//! \param  [out]  V              The computed eigenvectors 
//! \param  [in ]  e              The eigenvalues of the matrix
//! \param  [in ]  M              The matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_real_eigenvectors_from_eigenvalues(Rox_DynVec_Double e, Rox_ObjSet_Array2D_Double V, const Rox_Array2D_Double M);

//! @}

#endif // __OPENROX_REAL_EIGENVECTORS_FROM_EIGENVALUES__
