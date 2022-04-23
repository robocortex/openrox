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

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_cholesky_decomposition_inplace ( Rox_Array2D_Double S )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** L_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &L_data, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < cols; i++)
   {
      for(Rox_Sint j = i; j < cols; j++)
      {
         Rox_Double sum = L_data[i][j];

         for (Rox_Sint k = i - 1; k>= 0; k--)
         {
            sum -= L_data[i][k] * L_data[j][k];
         }

         if (i == j)
         {
            if (sum <= 0.0) 
            { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );}

            L_data[i][i] = sqrt(sum);
         }
         else
         {
            L_data[j][i] = sum / L_data[i][i];
         }
      }
   }

   // L must be a Lower triangular matrix : set to 0 the entries of the upper part of the matrix 
   for (Rox_Sint i = 0; i < cols; i++)
   {
      for (Rox_Sint j = 0; j < i; j++)
      {
         L_data[j][i] = 0.0;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_cholesky_decomposition ( Rox_Array2D_Double L, const Rox_Array2D_Double S )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_match_size ( L, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, S);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   if (cols != rows || cols < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** L_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &L_data, L );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy ( L, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // L is decomposed
   error = rox_array2d_double_cholesky_decomposition_inplace ( L );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
