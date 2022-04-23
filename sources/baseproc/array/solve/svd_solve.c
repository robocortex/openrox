//==============================================================================
//
//    OPENROX   : File svd_solve.c
//
//    Contents  : Implementation of svd_solve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "svd_solve.h"

#include <float.h>
#include <generated/array2d_double.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/minmax/minmax.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_svd_solve ( Rox_Array2D_Double x, const Rox_Array2D_Double A, const Rox_Array2D_Double b )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** dx   = NULL;
   Rox_Double ** dAtb = NULL;
   Rox_Double ** dS   = NULL;

   Rox_Array2D_Double AtA = NULL;
   Rox_Array2D_Double Atb = NULL;

   Rox_Array2D_Double U  = NULL;
   Rox_Array2D_Double V  = NULL;
   Rox_Array2D_Double S  = NULL;

   Rox_Double svd_min = 0.0;
   Rox_Double svd_max = 0.0;
   Rox_Double threshold = 0.0;

   if (!x || !A || !b) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint size = 0;
   
   error = rox_array2d_double_get_cols(&size, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&AtA, size, size); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&U, size, size);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&V, size, size);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Atb, size, 1);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&S, size, 1);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   error   = rox_array2d_double_get_data_pointer_to_pointer ( &dx, x );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dAtb, Atb );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error   = rox_array2d_double_get_data_pointer_to_pointer ( &dS, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(!dx || !dAtb || !dS) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_mulmattransmat(Atb, A, b);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(AtA, A, A);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_svd(U, S, V, AtA);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(x, U, Atb); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_minmax(&svd_min, &svd_max, S);  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO: The Matlab threshold is S->size*svd_max*DBL_EPSILON 
   // However our svd does not provide exactly the same results 
   // Thus, the threshold is temporarily divided by two !! 

   threshold = size * svd_max * DBL_EPSILON/2.0;

   for (Rox_Sint i = 0; i < size; i++)
   {
      if (dS[i][0] > threshold)
      {
         dAtb[i][0] = dx[i][0] / dS[i][0];
      }
      else
      {
         dAtb[i][0] = 0.0;
      }
   }

   error = rox_array2d_double_mulmatmat(x, V, Atb);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&AtA);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&Atb);
   return error;
}

Rox_ErrorCode rox_svd_solve_homogeneous_system ( Rox_Array2D_Double x, const Rox_Array2D_Double A)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double AtA = NULL;

   Rox_Array2D_Double U  = NULL;
   Rox_Array2D_Double V  = NULL;
   Rox_Array2D_Double S  = NULL;

   Rox_Double svd_min = 0.0;
   Rox_Double svd_max = 0.0;
   // Rox_Double threshold = 0.0;

   if (!x || !A ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint size = 0;
   
   error = rox_array2d_double_get_cols(&size, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&AtA, size, size); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&U, size, size);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&V, size, size);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&S, size, 1);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dx = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dx, x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dS, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dV = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dV, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(!dx || !dS) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_mulmattransmat(AtA, A, A);   
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_svd(U, S, V, AtA);   
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_minmax(&svd_min, &svd_max, S);  
   ROX_ERROR_CHECK_TERMINATE(error)

   // TODO: The Matlab threshold is S->size*svd_max*DBL_EPSILON 
   // However our svd does not provide exactly the same results 
   // Thus, the threshold is temporarily divided by two !! 

   // threshold = size * svd_max * DBL_EPSILON/2.0;

   // look for minimum index

   Rox_Sint min_index = 0;
   Rox_Double min_singv = DBL_MAX;

   for (Rox_Sint i = 0; i < size; i++)
   {
      if (dS[i][0] < min_singv)
      {
         min_singv = dS[i][0];
         min_index = i;
      }
   }

   // if (min_singv < threshold)

   // x = V[:][min_index]
   // copy column min_index of V into X
   dx[0][0] = dV[0][min_index];
   dx[1][0] = dV[1][min_index];
   dx[2][0] = dV[2][min_index];

function_terminate:
   rox_array2d_double_del(&AtA);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&S);
   return error;
}
