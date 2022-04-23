//==============================================================================
//
//    OPENROX   : File gradientsobel.c
//
//    Contents  : Implementation of gradientsobel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "gradientsobel.h"

#include <inout/system/errors_print.h>
#include <baseproc/maths/maths_macros.h>

Rox_ErrorCode rox_array2d_float_gradientsobel_nomask ( Rox_Array2D_Float gu, Rox_Array2D_Float gv, Rox_Array2D_Float image_gray )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!gu || !gv || !image_gray) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Check that gu and gv have the same size of image_gray image 
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, gu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gv, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 
   
   error = rox_array2d_float_check_size(image_gray, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   // Get pointers to data 
   Rox_Float ** dim = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dim, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Float ** dgu = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dgu, gu);
   ROX_ERROR_CHECK_TERMINATE ( error ); 
  
   Rox_Float ** dgv = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&dgv, gv);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   Rox_Sint u = 0, v = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(u, v)
#endif

   // Compute the gradient 
   for (v = 0; v < rows; v++)
   {
      Rox_Sint nv = v+1;
      Rox_Sint pv = v-1;

      for (u = 0; u < cols; u++)
      {
         // Set gradient to zero 
         dgu[v][u] = 0.0f;
         dgv[v][u] = 0.0f;

         if (v == 0 || v == rows - 1) continue; // Do not consider the borders ?
         if (u == 0 || u == cols - 1) continue; // Do not consider the borders ?

         Rox_Sint nu = u + 1;
         Rox_Sint pu = u - 1;

         // 8 casts to int are better than computing everything and then cast 
         // (tested on Neptune: Dell Precision 750, core i7)

         Rox_Float pvpu = dim[pv][pu];
         Rox_Float vpu  = dim[v ][pu];
         Rox_Float nvpu = dim[nv][pu];
         Rox_Float pvnu = dim[pv][nu];
         Rox_Float vnu  = dim[v ][nu];
         Rox_Float nvnu = dim[nv][nu];
         Rox_Float pvu  = dim[pv][u ];
         Rox_Float nvu  = dim[nv][u ];

         dgu[v][u] = (- pvpu - 2.0f * vpu - nvpu + pvnu + 2.0f * vnu + nvnu);
         dgv[v][u] = (- pvpu - 2.0f * pvu - pvnu + nvpu + 2.0f * nvu + nvnu);

         //dgu[v][u] = (- dim[pv][pu] - 2.0f * dim[v][pu] - dim[nv][pu] + dim[pv][nu] + 2.0f * dim[v][nu] + dim[nv][nu]);
         //dgv[v][u] = (- dim[pv][pu] - 2.0f * dim[pv][u] - dim[pv][nu] + dim[nv][pu] + 2.0f * dim[nv][u] + dim[nv][nu]);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_gradientsobel_nomask ( Rox_Array2D_Point2D_Sshort gradient, Rox_Image image_gray )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!gradient || !image_gray) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_point2d_sshort_get_size(&rows, &cols, gradient);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(image_gray, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dim = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dim, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort * dg = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &dg, gradient);
   ROX_ERROR_CHECK_TERMINATE ( error );

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
         dg[v][u].u = 0;
         dg[v][u].v = 0;

         if (v == 0 || v == rows - 1) continue;  // Do not consider the borders ?
         if (u == 0 || u == cols - 1) continue;  // Do not consider the borders ?

         Rox_Sint nu = u + 1;
         Rox_Sint pu = u - 1;

         // 8 casts to short are better than computing everything and then cast 
         // (tested on Neptune: Dell Precision 750, core i7)

         Rox_Sshort pvpu = dim[pv][pu];
         Rox_Sshort vpu  = dim[v ][pu];
         Rox_Sshort nvpu = dim[nv][pu];
         Rox_Sshort pvnu = dim[pv][nu];
         Rox_Sshort vnu  = dim[v ][nu];
         Rox_Sshort nvnu = dim[nv][nu];
         Rox_Sshort pvu  = dim[pv][u ];
         Rox_Sshort nvu  = dim[nv][u ];

         dg[v][u].u = (- pvpu - 2 * vpu - nvpu + pvnu + 2 * vnu + nvnu);
         dg[v][u].v = (- pvpu - 2 * pvu - pvnu + nvpu + 2 * nvu + nvnu);
      }
   }

function_terminate:

   return error;
}

Rox_ErrorCode rox_array2d_sint_gradientsobel_nomask(Rox_Array2D_Sint gu, Rox_Array2D_Sint gv, Rox_Image image_gray)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!gu || !gv || !image_gray) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Check that gu and gv have the same size of image_gray image 
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_sint_get_size(&rows, &cols, gu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_check_size(gv, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 
   
   error = rox_array2d_uchar_check_size(image_gray, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   // Get pointers to data 
   Rox_Uchar ** dim = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dim, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint  ** dgu = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &dgu, gu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint  ** dgv = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &dgv, gv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint u = 0, v = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(u, v)
#endif

   // Compute the gradient 
   for (v = 0; v < rows; v++)
   {
      Rox_Sint nv = v+1;
      Rox_Sint pv = v-1;

      for (u = 0; u < cols; u++)
      {
         // Set gradient to zero 
         dgu[v][u] = 0;
         dgv[v][u] = 0;

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

         dgu[v][u] = (- pvpu - 2 * vpu - nvpu + pvnu + 2 * vnu + nvnu);
         dgv[v][u] = (- pvpu - 2 * pvu - pvnu + nvpu + 2 * nvu + nvnu);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_gradientsobel_singlepoint_nomask(Rox_Point2D_Sshort gradient, Rox_Image image_gray, Rox_Sint u, Rox_Sint v)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check parameters
   if (!gradient || !image_gray)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (v == 0 || v == rows - 1 || u == 0 || u == cols - 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** dim = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dim, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (dim == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint nu = u + 1;
   Rox_Sint pu = u - 1;
   Rox_Sint nv = v + 1;
   Rox_Sint pv = v - 1;

   if (dim[pv] == NULL || dim[v] == NULL || dim[nv] == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // 8 casts to short are better than computing everything and then cast 
   // (tested on Neptune: Dell Precision 750, core i7)
   short pvpu = dim[pv][pu];
   short vpu  = dim[v ][pu];
   short nvpu = dim[nv][pu];
   short pvnu = dim[pv][nu];
   short vnu  = dim[v ][nu];
   short nvnu = dim[nv][nu];
   short pvu  = dim[pv][u ];
   short nvu  = dim[nv][u ];

   gradient->u = (- pvpu - 2 * vpu - nvpu + pvnu + 2 * vnu + nvnu);
   gradient->v = (- pvpu - 2 * pvu - pvnu + nvpu + 2 * nvu + nvnu);

function_terminate:

   return error;
}
