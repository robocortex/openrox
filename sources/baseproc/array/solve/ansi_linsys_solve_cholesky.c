//==============================================================================
//
//    OPENROX   : File ansi_linsys_solve_cholesky.c
//
//    Contents  : Implementation of ansi linsys_solve_cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_linsys_solve_cholesky.h"

// #include <baseproc/array/multiply/mulmatmat.h>
// #include <baseproc/array/multiply/mulmattransmat.h>
// #include <baseproc/array/inverse/lotinverse.h>
// #include <baseproc/array/decomposition/cholesky.h>
// #include <inout/system/errors_print.h>

#include <baseproc/array/inverse/ansi_lotinverse.h>
#include <baseproc/array/decomposition/ansi_cholesky.h>
#include <baseproc/array/multiply/ansi_mulmatmat.h>
#include <baseproc/array/multiply/ansi_mulmattransmat.h>

//#include <inout/numeric/array_print.h>

#include <string.h>

int rox_ansi_array_float_linsys_solve_cholesky ( float * x, float * S, int S_size, float * v )
{
   int error = 0;

   // Allocate memory
   float * L = (float *) malloc ( S_size * S_size * sizeof(float) );
   float * Li = (float *) malloc ( S_size * S_size * sizeof(float) );
   float * w = (float *) malloc ( S_size * sizeof(float) );

   // Copy S to L since we use an inplace function
   memcpy ( L, S, S_size * S_size * sizeof ( float ) );
   
   // rox_array_float_print_as_array2d ( S , 6, 6 );

   // Compute the lower triangular matrix L such that S = L * L^T 
   error = rox_ansi_array_float_cholesky_decomposition_inplace ( L, S_size );
   if (error) goto function_terminate;

   // rox_array_float_print_as_array2d ( L , 6, 6 );

   error = rox_ansi_array_float_lotinverse ( Li, L, S_size );
   if (error) goto function_terminate;

   // rox_array_float_print_as_array2d ( Li , 6, 6 );

   // Compute x = inv(Si) * v = inv ( L )^T * inv(L) * v

   // Compute w = inv(L) * v
   error = rox_ansi_array_float_mulmatmat ( w, S_size, 1, Li, v, S_size );
   if (error) goto function_terminate;

   // Compute x = inv(L)^T * w
   error = rox_ansi_array_float_mulmattransmat ( x, S_size, 1, Li, w, S_size );   
   if (error) goto function_terminate;

function_terminate:
   if (L != NULL) 
      free (L);
   if (Li != NULL) 
      free (Li);
   if (w != NULL) 
      free (w);

   return error;
}

