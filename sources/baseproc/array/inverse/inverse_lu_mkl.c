//==============================================================================
//
//    OPENROX   : File inverse_lu.c
//
//    Contents  : Implementation of inverse lu module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inverse_lu.h"
#include <float.h>
#include <mkl.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>
#include <system/time/timer.h>

Rox_ErrorCode rox_array2d_double_inverse_lu(Rox_Array2D_Double Mi, Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0;
   int * mkl_pivotArray = NULL;
   double * mkl_M = NULL;
   double * ptr = NULL;
   
   // Define timer to measure performances
   //Rox_Timer timer = NULL;
   //Rox_Double time = 0.0;

   // Init new timer
   //error = rox_timer_new(&timer);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_match_size(Mi, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&rows, &cols, M); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols != rows || cols < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   mkl_M = (double *) mkl_malloc(cols*rows*sizeof(double), 64);
   // int pivotArray[rows]; //since our matrix has rows
   mkl_pivotArray = (int *) mkl_malloc(rows*sizeof(int), 64);

   Rox_Double ** M_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &M_data, M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Mi_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Mi_data, Mi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set Mi matrix to 0
   error = rox_array2d_double_fillval ( Mi, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy the ROX matrix to MKL
   ptr = mkl_M;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         *ptr = M_data[i][j];
         ptr++;
      }
   }

   // rox_timer_start(timer);

   error = LAPACKE_dgetrf(CblasRowMajor, rows, cols, mkl_M, cols, mkl_pivotArray);  
   if (error)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = LAPACKE_dgetri(CblasRowMajor, cols, mkl_M, cols, mkl_pivotArray);
   if (error)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Display elapsed time
   //rox_timer_stop(timer);
   //rox_timer_get_elapsed_ms(&time, timer);
   //rox_log("time to inverse the matrix with LU MKL = %f (ms)\n", time);

   // Copy the MKL matrix to ROX
   ptr = mkl_M;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         Mi_data[i][j] = *ptr;
         ptr++;
      }
   }


function_terminate:

   mkl_free(mkl_M);
   mkl_free(mkl_pivotArray);

   return error;
}
