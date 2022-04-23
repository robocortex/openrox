//============================================================================
//
//    OPENROX   : File gradient_anglenorm.c
//
//    Contents  : Implementation of gradient_anglenorm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "gradient_anglenorm.h"
#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <inout/system/errors_print.h>
#include <baseproc/maths/base/basemaths.h>

// #define GRADIENT_SCALE_MIN 1e-12

Rox_ErrorCode rox_array2d_float_gradient_angle_norm_nomask (
   Rox_Array2D_Float angle,
   Rox_Array2D_Float norm,
   Rox_Array2D_Float gu,
   Rox_Array2D_Float gv,
   Rox_Float norm_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!angle || !norm || !gu || !gv)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, angle);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(norm, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gu, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gv, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dgu = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgu, gu);

   Rox_Float ** dgv = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgv, gv);
   
   Rox_Float ** dn = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dn, norm);

   Rox_Float ** da = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &da, angle);

   Rox_Sint i = 0, j = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(i, j)
#endif

   for (i = 0; i < rows; i++)
   {
      for (j = 0; j < cols; j++)
      {
         Rox_Float ix = dgu[i][j];
         Rox_Float iy = dgv[i][j];

         dn[i][j] = (Rox_Float) sqrt(ix*ix+iy*iy);

         if (dn[i][j] < norm_threshold)
         {
            dn[i][j] = 0;
            da[i][j] = 0;
            continue;
         }

         // da[i][j] = atan2(iy, ix);
         da[i][j] = fast_atan2f2 ( iy, ix );
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_gradient_angle_scale_nomask (
   Rox_Array2D_Float angle,
   Rox_Array2D_Float scale,
   Rox_Array2D_Float gu,
   Rox_Array2D_Float gv,
   Rox_Float norm_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!angle || !scale || !gu || !gv)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, angle);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(scale, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gu, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gv, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dgu = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgu, gu);
   Rox_Float ** dgv = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgv, gv);
   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &ds, scale);
   Rox_Float ** da = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &da, angle);

   Rox_Sint u = 0, v = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(u, v)
#endif

   for (v = 0; v < rows; v++)
   {
      for (u = 0; u < cols; u++)
      {
         Rox_Float Iu = dgu[v][u];
         Rox_Float Iv = dgv[v][u];

         ds[v][u] = Iu*Iu + Iv*Iv;

         if (ds[v][u] < norm_threshold)
         {
            ds[v][u] = 0;
            da[v][u] = 0;
            continue;
         }

         // da[i][j] = atan2(Iv, Iu);
         da[v][u] = fast_atan2f2 ( Iv, Iu );
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_sshort_angle_scale_float (
   Rox_Float * scale,
   Rox_Float * angle,
   const Rox_Point2D_Sshort image_gradient
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float ret_scale = 0.0, ret_angle = 0.0;

   // Check parameters
   if (!image_gradient)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float grad_u = (Rox_Float) image_gradient->u;
   Rox_Float grad_v = (Rox_Float) image_gradient->v;

   // The scale
   ret_scale = grad_u * grad_u + grad_v * grad_v;

   // The scale atan2 should be replaced by a lookup table for better performance
   ret_angle = atan2f ( grad_v, grad_u );

   *scale = ret_scale;
   *angle = ret_angle;

function_terminate:
   return error;
}
Rox_ErrorCode rox_point2d_sshort_angle_scale (
   Rox_Double * scale,
   Rox_Double * angle,
   const Rox_Point2D_Sshort image_gradient
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ret_scale = 0.0, ret_angle = 0.0;

   // Check parameters
   if (!image_gradient)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double grad_u = (double) image_gradient->u;
   Rox_Double grad_v = (double) image_gradient->v;

   // The scale
   ret_scale = grad_u * grad_u + grad_v * grad_v;

   // The scale atan2 should be replaced by a lookup table for better performance
   ret_angle = atan2(grad_v, grad_u);

   *scale = ret_scale;
   *angle = ret_angle;

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_sshort_angle_norm (
   Rox_Double * norm,
   Rox_Double * angle,
   const Rox_Point2D_Sshort image_gradient
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ret_norm = 0.0, ret_angle = 0.0;

   // Check parameters
   if (!image_gradient)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double grad_u = (double) image_gradient->u;
   Rox_Double grad_v = (double) image_gradient->v;

   // The norm
   ret_norm = sqrt(grad_u * grad_u + grad_v * grad_v);

   // The scale atan2 should be replaced by a lookup table for better performance
   ret_angle = atan2(grad_v, grad_u);

   *norm = ret_norm;
   *angle = ret_angle;

function_terminate:
   return error;
}

ROX_API Rox_ErrorCode rox_image_gradient_sobel_angle_scale_nomask_buffers (
   Rox_Float * angle,
   Rox_Uint * scale,
   Rox_Image image_gray,
   Rox_Uint scale_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint  cols = 0, rows = 0;
   
   if(!angle || !scale || !image_gray) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&rows, &cols, image_gray); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dim = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dim, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

// #ifdef ROX_USES_OPENMP
//    #pragma omp parallel for schedule(dynamic) private(u, v)
// #endif
   for (Rox_Sint v = 1; v < rows - 1; v++) // Ignore the borders start from v = 1
   {
      Rox_Sint nv = v+1;
      Rox_Sint pv = v-1;

      for (Rox_Sint u = 1; u < cols - 1; u++) // Ignore the borders start from u = 1
      {
         Rox_Uint idx = v * cols + u;

         //if (v == 0 || v == rows - 1) continue; // Do not consider the borders ?
         //if (u == 0 || u == cols - 1) continue; // Do not consider the borders ?

         Rox_Sint nu = u + 1;
         Rox_Sint pu = u - 1;

         // Compute Sobel image gradient
         // 8 casts to int are better than computing everything and then cast
         // (tested on Neptune: Dell Precision 750, core i7)

         Rox_Sint pvpu = dim[pv][pu];
         Rox_Sint vpu  = dim[v ][pu];
         Rox_Sint nvpu = dim[nv][pu];
         Rox_Sint pvnu = dim[pv][nu];
         Rox_Sint vnu  = dim[v ][nu];
         Rox_Sint nvnu = dim[nv][nu];
         Rox_Sint pvu  = dim[pv][u ];
         Rox_Sint nvu  = dim[nv][u ];

         Rox_Sint dx = (- pvpu - 2 * vpu - nvpu + pvnu + 2 * vnu + nvnu);
         Rox_Sint dy = (- pvpu - 2 * pvu - pvnu + nvpu + 2 * nvu + nvnu);

         // Compute scale of gradient (scale = square of scale)
         Rox_Uint scale_val = dx*dx + dy*dy;

         if (scale_val < scale_threshold)
         {
            scale[idx] = 0;
            angle[idx] = 0;
            continue;
         }

         // Compute angle of gradient
         Rox_Float angle_val = (Rox_Float) quad_arctan2((Rox_Float) dy, (Rox_Float) dx);

         // Store scale and gradient
         scale[idx] = scale_val;
         angle[idx] = angle_val;
      }
   }

function_terminate:

   return error;
}

Rox_ErrorCode rox_array2d_sint_gradient_angle_scale_nomask (
   Rox_Array2D_Float angle,
   Rox_Array2D_Uint scale,
   Rox_Array2D_Sint gu,
   Rox_Array2D_Sint gv,
   Rox_Uint norm_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!angle || !scale || !gu || !gv)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, angle);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(scale, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_check_size(gu, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_check_size(gv, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint ** dgu = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &dgu, gu);

   Rox_Sint ** dgv = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &dgv, gv);

   Rox_Uint ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &ds, scale);
   
   Rox_Float ** da = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &da, angle);
   
   Rox_Sint i = 0, j = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(i, j)
#endif
   for (i = 0; i < rows; i++)
   {
      for (j = 0; j < cols; j++)
      {
         Rox_Sint ix = dgu[i][j];
         Rox_Sint iy = dgv[i][j];

         ds[i][j] = ix*ix+iy*iy;

         if (ds[i][j] < norm_threshold)
         {
            ds[i][j] = 0;
            da[i][j] = 0;
            continue;
         }

         // da[i][j] = atan2(iy, ix);
         da[i][j] = (Rox_Float) fast_atan2f2((Rox_Float) iy, (Rox_Float) ix);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_gradient_sobel_angle_scale_nomask (
   Rox_Array2D_Float angle,
   Rox_Array2D_Uint  scale,
   Rox_Image image_gray,
   Rox_Sint scale_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!angle || !scale || !image_gray)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, angle);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(scale, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint  ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &ds, scale);

   Rox_Float ** da = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &da, angle);
   
   Rox_Uchar ** dim = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dim, image_gray);

   Rox_Sint u = 0, v = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(u, v)
#endif
   for (v = 0; v < rows; v++)
   {
      Rox_Sint nv = v+1;
      Rox_Sint pv = v-1;

      for (u = 0; u < cols; u++)
      {
         if (v == 0 || v == rows - 1) continue; // Do not consider the borders ?
         if (u == 0 || u == cols - 1) continue; // Do not consider the borders ?

         Rox_Sint nu = u + 1;
         Rox_Sint pu = u - 1;

         // 8 casts to int are better than computing everything and then cast
         // (tested on Neptune: Dell Precision 750, core i7)

         Rox_Sint pvpu = dim[pv][pu];
         Rox_Sint vpu  = dim[v ][pu];
         Rox_Sint nvpu = dim[nv][pu];
         Rox_Sint pvnu = dim[pv][nu];
         Rox_Sint vnu  = dim[v ][nu];
         Rox_Sint nvnu = dim[nv][nu];
         Rox_Sint pvu  = dim[pv][u ];
         Rox_Sint nvu  = dim[nv][u ];

         Rox_Sint Iu = (- pvpu - 2 * vpu - nvpu + pvnu + 2 * vnu + nvnu);
         Rox_Sint Iv = (- pvpu - 2 * pvu - pvnu + nvpu + 2 * nvu + nvnu);

         ds[v][u] = (Rox_Uint) (Iu*Iu + Iv*Iv);

         if (ds[v][u] < (Rox_Uint) scale_threshold)
         {
            ds[v][u] = 0;
            da[v][u] = 0;
            continue;
         }

         // da[i][j] = atan2(Iv, Iu);
         da[v][u] = (Rox_Float) fast_atan2f2((Rox_Float) Iv, (Rox_Float) Iu);
      }
   }

function_terminate:
   return error;

}
