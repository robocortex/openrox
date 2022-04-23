//==============================================================================
//
//    OPENROX   : File real_eigenvalues_from_tridiagonal_matrix.c
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

#include "real_eigenvalues_from_tridiagonal_matrix.h"

#include <math.h> 

#include <baseproc/array/tridiagonal/tridiagonal.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_real_eigenvalues_from_tridiagonal_matrix(Rox_DynVec_Double e, const Rox_Array2D_Double T)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint is_tridiagonal = 0;
   Rox_Sint rows = 0;
   Rox_Uint k = 0;
   Rox_Double eigenvalue = 0.0;

   if (!e || !T ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_is_tridiagonal(&is_tridiagonal, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (is_tridiagonal == 0)
   { error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_rows(&rows, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Td = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Td, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   while (k < (Rox_Uint) (rows-1))
   {
      if ( fabs(Td[k+1][k]) < 10e-8 )
      {
         // real eigenvalue
         eigenvalue = Td[k][k];

         error = rox_dynvec_double_append(e, &eigenvalue);
         ROX_ERROR_CHECK_TERMINATE ( error );

         k = k + 1;
      }
      else
      {
         // skip the two complex conjugate eigenvalues
         k = k + 2;
      }
   }

   // check for the last
   if (k == (rows-1))
   {
      // real eigenvalue
      eigenvalue = Td[k][k];

      error = rox_dynvec_double_append(e, &eigenvalue);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   return error;
}
