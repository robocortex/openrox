//==============================================================================
//
//    OPENROX   : File lotinverse_mkl.c
//
//    Contents  : Implementation MKL of inverse module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "lotinverse.h"

#include <mkl.h>
#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_lotinverse(Rox_Array2D_Double Li, Rox_Array2D_Double L)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   double * mkl_L = NULL;
   double * ptr = NULL;
   
   error = rox_array2d_double_match_size(Li, L);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, L); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols != rows || cols < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   mkl_L = (double *) mkl_malloc(cols*rows*sizeof(double), 64);
   
   Rox_Double ** di = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &di, L );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dd, Li );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set Li matrix to 0
   error = rox_array2d_double_fillval ( Li, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ptr = mkl_L;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         *ptr = di[i][j];
         ptr++;
      }
   }

   int res = LAPACKE_dtrtri(CblasRowMajor, 'L', 'N', cols, mkl_L, cols);
   if (res)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptr = mkl_L;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dd[i][j] = *ptr;
         ptr++;
      }
   }

//   for (Rox_Sint i = 0; i < rows; i++)
//   {
//      for (Rox_Sint j = i; j < cols; j++)
//      {
//         dd[i][j] = dd[j][i];
//      }
//   }

function_terminate:

   mkl_free(mkl_L);

   return error;
}
