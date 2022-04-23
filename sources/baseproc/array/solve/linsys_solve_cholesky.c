//==============================================================================
//
//    OPENROX   : File linsys_solve_cholesky.c
//
//    Contents  : Implementation of linsys_solve_cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_solve_cholesky.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/inverse/lotinverse.h>
#include <baseproc/array/decomposition/cholesky.h>
#include <inout/system/errors_print.h>

//#include <inout/numeric/array_print.h>

#include <string.h>

Rox_ErrorCode rox_linsys_solve_cholesky ( Rox_Array2D_Double x, const Rox_Array2D_Double S, const Rox_Array2D_Double v )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double L = NULL;
   Rox_Array2D_Double Li = NULL;
   Rox_Array2D_Double w = NULL;

   if ( !x || !S || !v ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint size = 0;
   error = rox_array2d_double_get_cols ( &size, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &Li, size, size ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &L, size, size ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &w, size, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the lower triangular matrix L such that S = L * L^T 
   error = rox_array2d_double_cholesky_decomposition ( L, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_lotinverse ( Li, L );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute x = inv(Si) * v = inv ( L )^T * inv(L) * v

   // Compute w = inv(L) * v
   error = rox_array2d_double_mulmatmat ( w, Li, v ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Compute x = inv(L)^T * w
   error = rox_array2d_double_mulmattransmat ( x, Li, w );   
   ROX_ERROR_CHECK_TERMINATE ( error );


function_terminate:
   rox_array2d_double_del(&Li);
   rox_array2d_double_del(&L);
   rox_array2d_double_del(&w);
   return error;
}
