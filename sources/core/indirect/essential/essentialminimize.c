//==============================================================================
//
//    OPENROX   : File essentialminimize.c
//
//    Contents  : API of essentialminimize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "essentialminimize.h"

#include <float.h>
#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/scale/scale.h>

#include <core/indirect/essential/essentialerror.h>
#include <baseproc/calculus/linsys/linsys_essential_geometric_weighted_premul.h>

#include <inout/system/errors_print.h>

#define ROX_ESSENTIAL_MINIMIZE_ITERATIONS 20

Rox_ErrorCode rox_essential_minimize (
   Rox_MatSE3 pose, 
   Rox_DynVec_Uint mask, 
   Rox_DynVec_Point2D_Float ref, 
   Rox_DynVec_Point2D_Float cur
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double E = NULL, vecerr = NULL;
   Rox_Array2D_Double JtJ = NULL, Jtf = NULL, invJtJ = NULL, x = NULL, rx = NULL;
   Rox_Array2D_Double dist = NULL, weight = NULL, weightbuf1 = NULL, weightbuf2 = NULL;
   Rox_Uint k = 0, maxtransid = 0, pos = 0, idpt = 0;
   Rox_Double maxtrans = 0.0, ctrans = 0.0;
   Rox_Double normalizertrans = 0.0;

   if (!pose || !ref || !cur) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (ref->used != cur->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&E, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&vecerr, ref->used * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&dist, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&weight, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&weightbuf1, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&weightbuf2, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&JtJ, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Jtf, 5, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&invJtJ, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 5, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&rx, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_dynvec_uint_reset(mask);

   Rox_Double *de = NULL;
   error = rox_array2d_double_get_data_pointer(&de, vecerr);
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double *dw = NULL;
   error = rox_array2d_double_get_data_pointer(&dw, weight);
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double *dd = NULL;
   error = rox_array2d_double_get_data_pointer(&dd, dist);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double **dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double **dx = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx, x);
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double **drx = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&drx, rx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint iter = 0; iter < ROX_ESSENTIAL_MINIMIZE_ITERATIONS; iter++)
   {
      // Search for the biggest translation direction
      maxtrans = 0;
      maxtransid = 0;
      for (k = 0; k < 3; k++)
      {
         ctrans = fabs(dt[k][3]);
         if (ctrans > maxtrans)
         {
            maxtrans = ctrans;
            maxtransid = k;
         }
      }

      // We normalize using the translation, need to be non zero
      if (maxtrans < DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }

      // Keep the sign
      normalizertrans = maxtrans;
      if (dt[k][3] < 0.0)
      {
         normalizertrans = -maxtrans;
      }

      // Normalize
      for (k = 0; k < 3; k++)
      {
         dt[k][3] = dt[k][3] / normalizertrans;
      }

      error = rox_transformtools_build_essential ( E, pose );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_essential_geometric_error(vecerr, E, ref, cur);
      ROX_ERROR_CHECK_TERMINATE(error)

      for (idpt = 0; idpt < ref->used; idpt++)
      {
         dd[idpt] = de[idpt*2]*de[idpt*2] + de[idpt*2+1]*de[idpt*2+1];
      }

      error = rox_array2d_double_tukey(weight, weightbuf1, weightbuf2, dist);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_jacobian_essential_geometric_weighted_premul(JtJ, Jtf, vecerr, weight, pose, E, ref->data, cur->data, ref->used, maxtransid);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_svdinverse(invJtJ, JtJ);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_mulmatmat(x, invJtJ, Jtf);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Create a classic update vector for se(3) but ignoring the biggest translation (otherwise the solution is always t=0)
      pos = 0;
      for (k = 0; k < 6; k++)
      {
         if (k == maxtransid)
         {
            drx[k][0] = 0;
         }
         else
         {
            drx[k][0] = dx[pos][0];
            pos++;
         }
      }

      error = rox_array2d_double_scale_inplace(rx, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_matse3_update_left(pose, rx);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Set outlier flag using last weight
   for (idpt = 0; idpt < ref->used; idpt++)
   {
      Rox_Uint m;
      m = 0;

      if (dw[idpt] > DBL_EPSILON)
      {
         m = 1;
      }

      rox_dynvec_uint_append(mask, &m);
   }

function_terminate:
   rox_array2d_double_del(&vecerr);
   rox_array2d_double_del(&dist);
   rox_array2d_double_del(&weight);
   rox_array2d_double_del(&weightbuf1);
   rox_array2d_double_del(&weightbuf2);
   rox_array2d_double_del(&JtJ);
   rox_array2d_double_del(&invJtJ);
   rox_array2d_double_del(&Jtf);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&rx);
   rox_array2d_double_del(&E);

   return error;
}
