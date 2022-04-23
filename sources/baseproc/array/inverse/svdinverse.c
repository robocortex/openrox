//==============================================================================
//
//    OPENROX   : File svdinverse.c
//
//    Contents  : Implementation of svdinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "svdinverse.h"

#include <float.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_array2d_double_svdinverse
(
   Rox_Array2D_Double dest,
   Rox_Array2D_Double source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double U = NULL, S = NULL, V = NULL, fulliS = NULL, part = NULL;
   Rox_Double smax = 0.0, smin = 0.0, threshold = 0.0;

   if (!dest || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_match_size((Rox_Array2D) dest, (Rox_Array2D) source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Size vals = (cols < rows) ? cols : rows;

   error = rox_array2d_double_new(&U, rows, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&V, cols, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&S, (Rox_Sint) vals, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&part, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svd(U, S, V, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dS, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&fulliS, cols, rows);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute full iS matrix
   Rox_Double ** dFS = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dFS, fulliS );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(fulliS, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   smin = +DBL_MAX;
   smax = -DBL_MAX;
   for (Rox_Sint i = 0; i < vals; i++)
   {
      // if (dS[i][0] < smin && dS[i][0] > DBL_EPSILON) smin = dS[i][0];
      if (dS[i][0] < smin) smin = dS[i][0];
      if (dS[i][0] > smax) smax = dS[i][0];
   }

   if (fabs(smin) < DBL_EPSILON)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   threshold = vals * smax * DBL_EPSILON / 2.0;

   for (Rox_Sint i = 0; i < vals; i++)
   {
      if (fabs(dS[i][0]) < threshold) dFS[i][i] = 0.0;
      else dFS[i][i] = 1.0 / dS[i][0];
   }

   error = rox_array2d_double_mulmatmattrans(part, fulliS, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(dest, V, part);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&part);
   rox_array2d_double_del(&fulliS);

   return error;
}
