//==============================================================================
//
//    OPENROX   : File real_eigenvalues_eigenvectors_mkl.c
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
#include <inout/system/errors_print.h>
#include <mkl.h>
#include <float.h>
#include <math.h>

Rox_ErrorCode rox_real_eigenvalues_eigenvectors(Rox_DynVec_Double e, Rox_ObjSet_Array2D_Double V, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0;
   double * mkl_M  = NULL;
   double * mkl_er = NULL; // real part of eigenvalues
   double * mkl_ei = NULL; // imag part of eigenvalues
   double * mkl_vr = NULL; // right eigenvectors
   double * mkl_vl = NULL; // left  eigenvectors
   double * ptr    = NULL;
   double * ptr_er = NULL;
   double * ptr_ei = NULL;
   
   Rox_Array2D_Double v = NULL;

   if (!e || !V || !M ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_size(&rows, &cols, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   mkl_M  = (double *) mkl_malloc(cols*rows*sizeof(double), 64);
   mkl_vr = (double *) mkl_malloc(cols*rows*sizeof(double), 64);
   mkl_er = (double *) mkl_malloc(cols*sizeof(double), 64);
   mkl_ei = (double *) mkl_malloc(cols*sizeof(double), 64);

   Rox_Double ** M_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &M_data, M );
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

   error = LAPACKE_dgeev(CblasRowMajor, 'N', 'V', cols, mkl_M, cols, mkl_er, mkl_ei, mkl_vl, cols, mkl_vr, cols);  
   if (error)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Copy the MKL output to ROX containers
   ptr_er = mkl_er;
   ptr_ei = mkl_ei;
   for (Rox_Sint i = 0; i < cols; i++)
   {
      Rox_Double eigenvalue_real = *ptr_er;
      Rox_Double eigenvalue_imag = *ptr_ei;
      
      ptr_er++;
      ptr_ei++;

      // test if eigenvector is real (store it only if real)
      if (fabs(eigenvalue_imag) < FLT_EPSILON)
      {
         error = rox_dynvec_double_append(e, &eigenvalue_real);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_new(&v, rows, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );
      
         Rox_Double ** vd = NULL;
         error = rox_array2d_double_get_data_pointer_to_pointer( &vd, v );
         ROX_ERROR_CHECK_TERMINATE ( error );

         ptr = mkl_vr + i;
         for (Rox_Sint j = 0; j < rows; j++)
         {
            vd[j][0] = *ptr;
            ptr = ptr + cols;
         }

         error = rox_objset_array2d_double_append(V, v);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:
   return error;
}


