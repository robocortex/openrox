//==============================================================================
//
//    OPENROX   : File svdsort.c
//
//    Contents  : Implementation of svdsort module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "svd.h"
#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <inout/system/errors_print.h>

#include <mkl.h>

Rox_ErrorCode rox_array2d_double_svd(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V, const Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   double *mkl_a = NULL, *mkl_u = NULL, *mkl_s = NULL, *mkl_v = NULL, *mkl_b = NULL, *ptr = NULL;

   if (!U || !V || !S || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint uw, uh;
   error = rox_array2d_double_get_size(&uh, &uw, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint vw, vh;
   error = rox_array2d_double_get_size(&vh, &vw, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint sw, sh;
   error = rox_array2d_double_get_size(&sh, &sw, S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint iw, ih;
   error = rox_array2d_double_get_size(&ih, &iw, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (sw != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (uh != ih) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (uw != ih) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (vh != iw) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (vw != iw) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   //if (sh != ROX_MIN(ih, iw)) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (sh != ih) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (ih < iw) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Double ** dU = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dU, U);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dV = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dV, V);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dS = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS, S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dM = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dM, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   mkl_a = (double *)mkl_malloc(ih*iw*sizeof(double), 64);
   mkl_s = (double *)mkl_malloc(ROX_MIN(ih,iw)*sizeof(double), 64);
   mkl_u = (double *)mkl_malloc(ih*ih*sizeof(double), 64);
   mkl_v = (double *)mkl_malloc(iw*iw*sizeof(double), 64);
   mkl_b = (double *)mkl_malloc(ROX_MIN(ih,iw)*sizeof(double), 64);

   ptr = mkl_a;
   for (Rox_Sint i = 0; i < ih; i++)
   {
      for (Rox_Sint j = 0; j < iw; j++)
      {
         *ptr = dM[i][j];
         ptr++;
      }
   }

   int res = LAPACKE_dgesvd(CblasRowMajor, 'A', 'A', ih, iw, mkl_a, iw, mkl_s, mkl_u, ih, mkl_v, iw, mkl_b);
   if (res)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptr = mkl_u;
   for (Rox_Sint i = 0; i < ih; i++)
   {
      for (Rox_Sint j = 0; j < ih; j++)
      {
         dU[i][j] = *ptr;
         ptr++;
      }
   }

   ptr = mkl_v;
   for (Rox_Sint i = 0; i < iw; i++)
   {
      for (Rox_Sint j = 0; j < iw; j++)
      {
         dV[j][i] = *ptr;
         ptr++;
      }
   }

   ptr = mkl_s;
   for (Rox_Sint i = 0; i < ROX_MIN(ih,iw); i++)
   {  
      dS[i][0] = *ptr;
      ptr++;
   }

function_terminate:

   mkl_free(mkl_a);
   mkl_free(mkl_s);
   mkl_free(mkl_u);
   mkl_free(mkl_v);
   mkl_free(mkl_b);

   return error;
}
