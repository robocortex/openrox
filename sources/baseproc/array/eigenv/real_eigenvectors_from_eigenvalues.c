//==============================================================================
//
//    OPENROX   : File real_eigenvectors_from_eigenvalues.c
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

#include "real_eigenvectors_from_eigenvalues.h"
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_real_eigenvectors_from_eigenvalues(Rox_DynVec_Double e, Rox_ObjSet_Array2D_Double V, const Rox_Array2D_Double Q)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   Rox_Array2D_Double I = NULL;
   Rox_Array2D_Double M = NULL;

   Rox_Array2D_Double Ud = NULL;
   Rox_Array2D_Double Sd = NULL;
   Rox_Array2D_Double Vd = NULL;
   Rox_Array2D_Double cd = NULL;

   if (!e || !V || !Q ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * eigenvalues = NULL;
   error = rox_dynvec_double_get_data_pointer(&eigenvalues, e);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_used = 0;
   error = rox_dynvec_double_get_used(&nb_used, e);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&rows, &cols, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&I, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&M, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Ud, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Vd, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Sd, rows, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=0; k <nb_used; k++)
   {
      // M = Q - e * I

      error = rox_array2d_double_fillunit(I);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_scale_inplace(I, eigenvalues[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_substract(M, Q, I);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_svd(Ud, Sd, Vd, M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_svd_sort_SV(Sd, Vd);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&cd, rows, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_col(cd, Vd, cols-1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // V(:,k) = Vd(:,end)
      error = rox_objset_array2d_double_append(V, cd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   rox_array2d_double_del(&I);
   rox_array2d_double_del(&M);
   rox_array2d_double_del(&Ud);
   rox_array2d_double_del(&Sd);
   rox_array2d_double_del(&Vd);

   return error;
}