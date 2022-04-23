//==============================================================================
//
//    OPENROX   : File canny.c
//
//    Contents  : Implementation of canny module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "canny.h"

#include <float.h>
#include <generated/array2d_float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/convolve/array2d_float_symmetric_separable_convolve.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/image/gradient/gradient_anglenorm.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_canny_trace(Rox_Uchar ** dr, Rox_Float ** db, Rox_Sint j, Rox_Sint i, Rox_Float low, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (dr[i][j] != 0) goto function_terminate;

   dr[i][j] = 255;

   for (Rox_Sint dy = -1; dy <= 1; dy++)
   {
      Rox_Sint cy = i + dy;
      if (cy < 0 || cy >= rows) continue;

      for (Rox_Sint dx = -1; dx <= 1; dx++)
      {
         Rox_Sint cx = j + dx;
         if (cx < 0 || cx >= rows) continue;

         if (dy == 0 && dx == 0) continue;

         if (db[cy][cx] > low)
         {
            rox_canny_trace(dr, db, cx, cy, low, cols, rows);
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_canny_process(Rox_Image result, Rox_Image source, Rox_Double sigma, Rox_Float low, Rox_Float high)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint i,j;
   Rox_Sint cols, rows;
   Rox_Array2D_Float sourcef = NULL, filtered = NULL, gx = NULL, gy = NULL, ga = NULL, gn = NULL, buf = NULL;
   Rox_Array2D_Float hfilter = NULL, vfilter = NULL;
   Rox_Float **da, **dn, **db;
   Rox_Uchar **dr;
   Rox_Float angle, mag;

	Rox_Float step1 = (Rox_Float) (ROX_PI / 8.0f);
   Rox_Float step2 = (Rox_Float) (3.0 * ROX_PI / 8.0f);
   Rox_Float step3 = (Rox_Float) (5.0 * ROX_PI / 8.0f);
   Rox_Float step4 = (Rox_Float) (7.0 * ROX_PI / 8.0f);

   if (!source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(result, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&sourcef, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&filtered, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&gx, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&gy, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ga, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&gn, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&buf, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_from_uchar(sourcef, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (sigma < DBL_EPSILON)
   {
      error = rox_array2d_float_copy(filtered, sourcef);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_kernelgen_gaussian2d_separable_float_new(&hfilter, &vfilter, (Rox_Float) sigma, 4.0f);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_symmetric_seperable_convolve(filtered, sourcef, hfilter);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_float_gradientsobel_nomask(gx, gy, filtered);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_gradient_angle_norm_nomask(ga, gn, gx, gy, 1e-12f);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_fillval(buf, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_fillval(result, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer(&da, ga);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer(&dn, gn);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer(&db, buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_data_pointer_to_pointer(&dr, result);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (i = 1; i < (Rox_Uint) (rows - 1); i++)
   {
      for (j = 1; j < (Rox_Uint) (cols - 1); j++)
      {
         angle = da[i][j];
         mag = dn[i][j];

         if (angle < 0.0)
         {
            angle = (Rox_Float) (ROX_PI + angle);
         }

         if (angle <= step1 || angle >= step4)
         {
            if (mag > dn[i][j - 1] && mag > dn[i][j + 1])
            {
               db[i][j] = mag;
            }
         }
         else if (angle < step2)
         {
            if (mag > dn[i - 1][j - 1] && mag > dn[i + 1][j + 1])
            {
               db[i][j] = mag;
            }
         }
         else if (angle < step3)
         {
            if (mag > dn[i - 1][j] && mag > dn[i + 1][j])
            {
               db[i][j] = mag;
            }
         }
         else
         {
            if (mag > dn[i - 1][j + 1] && mag > dn[i + 1][j - 1])
            {
               db[i][j] = mag;
            }
         }
      }
   }

   for (i = 1; i < (Rox_Uint) (rows - 1); i++)
   {
      for (j = 1; j < (Rox_Uint) (cols - 1); j++)
      {
         if (db[i][j] >= high)
         {
            rox_canny_trace(dr, db, j, i, low, cols, rows);
         }
      }
   }

function_terminate:

   rox_array2d_float_del(&sourcef);
   rox_array2d_float_del(&gx);
   rox_array2d_float_del(&gy);
   rox_array2d_float_del(&ga);
   rox_array2d_float_del(&gn);
	rox_array2d_float_del(&buf);
   rox_array2d_float_del(&hfilter);
   rox_array2d_float_del(&vfilter);
   rox_array2d_float_del(&filtered);

   return error;
}
