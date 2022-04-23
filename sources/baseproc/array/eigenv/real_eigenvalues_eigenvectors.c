//==============================================================================
//
//    OPENROX   : File real_eigenvalues_eigenvectors.c
//
//    Contents  : Implementation of matse3 from points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "real_eigenvalues_eigenvectors.h"

#include <baseproc/array/tridiagonal/tridiagonal.h>
#include <baseproc/array/eigenv/real_eigenvalues_from_tridiagonal_matrix.h>
#include <baseproc/array/eigenv/real_eigenvectors_from_eigenvalues.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_real_eigenvalues_eigenvectors(Rox_DynVec_Double e, Rox_ObjSet_Array2D_Double V, const Rox_Array2D_Double Q)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double T = NULL;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   if (!e || !V || !Q ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_size(&rows, &cols, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&T , rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_tridiagonal(T, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_real_eigenvalues_from_tridiagonal_matrix(e, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_real_eigenvectors_from_eigenvalues(e, V, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&T);

   return error;
}
