//==============================================================================
//
//    OPENROX   : File cholesky.c
//
//    Contents  : Implementation of cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cholesky.h"

#include <baseproc/maths/maths_macros.h>
#include <mkl.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_cholesky_decomposition (
   Rox_Array2D_Double L, 
   const Rox_Array2D_Double input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   double * mkl_input = NULL;
   double *ptr;
   
   error = rox_array2d_double_match_size(L, input);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint width = 0, height = 0;
   error = rox_array2d_double_get_size(&height, &width, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (width != height || width < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   mkl_input = (double *) mkl_malloc(width*height*sizeof(double), 64);

   Rox_Double ** dl = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dl, L );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** di = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &di, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ptr = mkl_input;
   for ( Rox_Sint i = 0; i < height; i++)
   {
      for ( Rox_Sint j = 0; j < width; j++)
      {
         *ptr = di[i][j];
         ptr++;
      }
   }

   int res = LAPACKE_dpotrf(CblasRowMajor, 'L', width, mkl_input, width);
   if (res)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptr = mkl_input;
   for ( Rox_Sint i = 0; i < height; i++)
   {
      for ( Rox_Sint j = 0; j < width; j++)
      {
         if(j <= i)
         {
            dl[i][j] = *ptr;
         }
         else
         {
            dl[i][j] = 0.0;
         }
         ptr++;
      }
   }
   
function_terminate:

   mkl_free(mkl_input);

   return error;
}