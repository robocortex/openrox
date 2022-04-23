//============================================================================
//
//    OPENROX   : File gaussian2d.c
//
//    Contents  : Implementation of gaussian2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "gaussian2d.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_kernelgen_gaussian2d_separable_float_new(Rox_Array2D_Float *hfilter, Rox_Array2D_Float *vfilter, Rox_Float sigma, Rox_Float cutoff)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float H = NULL, V = NULL;

   if (!hfilter) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!vfilter) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *hfilter = NULL;
   *vfilter = NULL;

   if (fabs(sigma) < DBL_EPSILON * 2.0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute optimal area
   Rox_Sint radius = abs((Rox_Sint)(2.0 * (sigma * cutoff) + 1.0) / 2);
   Rox_Sint width = 2 * radius + 1;

   // A check to disable bad input
   if (width > 100) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_new(&H, 1, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&V, width, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dh = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dh, H);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dv = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dv, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float sum = 0.0;
   for (Rox_Sint  i = -radius; i <= radius; i++)
   {
      dh[0][i + radius] = (Rox_Float) exp(-1.0 * (((Rox_Float)(i * i)) / (2.0 * (sigma * sigma))));
      sum += dh[0][i + radius];
   }

   for (Rox_Sint i = -radius; i <= radius; i++)
   {
      dh[0][i + radius] /= sum;
      dv[i + radius][0] = dh[0][i + radius];
   }

   *hfilter = H;
   *vfilter = V;

function_terminate:
   return error;
}
