//==============================================================================
//
//    OPENROX   : File tridiagonal.c
//
//    Contents  : Implementation of tridiagonal module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tridiagonal.h"

#include <math.h>

#include <baseproc/array/decomposition/qr.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_tridiagonal(Rox_Array2D_Double T, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint convergence = 0;
   Rox_Sint iter = 0;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   Rox_Array2D_Double R = NULL;
   Rox_Array2D_Double Q = NULL;
   Rox_Array2D_Double RQ = NULL;

   if (!T || !M) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_size(&rows, &cols, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&R, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Q, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_copy(&RQ, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   while ((convergence==0) && (iter < 10000))
   {
      error = rox_array2d_double_qr(Q, R, RQ);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(RQ, R, Q);
      ROX_ERROR_CHECK_TERMINATE ( error );

      iter = iter + 1;
   }

   error = rox_array2d_double_copy(T, RQ);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&Q);
   rox_array2d_double_del(&RQ);

   return error;
}

Rox_ErrorCode rox_array2d_double_is_tridiagonal(Rox_Sint * convergence, const Rox_Array2D_Double T)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint rows = 0;
   Rox_Uint cont = 0;

   if (!convergence || !T ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_rows(&rows, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Td = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Td, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double n = (double) rows;

   Rox_Double score = 0.0;

   for (Rox_Sint r = 2; r < rows; r++) 
   {
      for (Rox_Sint c = 0; c < r-2; c++)
      {
         score = score + fabs(Td[r][c]);
         cont = cont + 1;
      }
   }

   score = score / ((n-1)*(n-2)/2);

   if (score < 10e-8)
   {
      *convergence = 1;
   }
   else
   {
      *convergence = 0;
   }

function_terminate:

   return error;
}