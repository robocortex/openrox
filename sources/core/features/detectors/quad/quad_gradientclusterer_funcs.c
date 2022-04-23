//==============================================================================
//
//    OPENROX   : File quad_gradientclusterer.c
//
//    Contents  : Implementation of quad_gradientclusterer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quad_gradientclusterer.h"

#include <generated/array2d_uint.h>
#include <generated/array2d_sint.h>

// Verification of computations
#include <generated/array2d_point2d_sshort.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/base/basemaths.h>
#include <baseproc/image/image.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/image/gradient/gradient_anglenorm.h>
#include <baseproc/image/imask/fill/set_data.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

//#define SQUARE_SCALE_GRADIENT_MIN 60*60 // 14400
//#define SQUARE_SCALE_GRADIENT_MAX 120*120 // 57600

//#define SQUARE_SCALE_GRADIENT_MIN 120*120 // 14400
//#define SQUARE_SCALE_GRADIENT_MAX 180*180 // 57600


//#define SQUARE_SCALE_GRADIENT_MIN 240*240 // 14400
//#define SQUARE_SCALE_GRADIENT_MAX 260*260 // 57600

//#define SQUARE_SCALE_GRADIENT_MIN 240*240 // 14400
//#define SQUARE_SCALE_GRADIENT_MAX 240*240 // 57600

//#define SQUARE_SCALE_GRADIENT_MIN 120*120 // 14400
//#define SQUARE_SCALE_GRADIENT_MAX 180*180 // 57600


#define SQUARE_SCALE_GRADIENT_MIN 100*100 // 14400
#define SQUARE_SCALE_GRADIENT_MAX 140*140 // 57600

// #define NOFILTER_ISOLATED_PIXELS

// Compute filtered scale and angle of the image gradient
// TODO : verify if this function is redundant relative to functions in "gradient_anglenorm.c"
Rox_ErrorCode rox_gradientclusterer_buildgradients (
   Rox_Array2D_Uint Imagval,
   Rox_Uint * Imag,
   Rox_Float * Itheta,
   Rox_Image image_gray,
   Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idx = 0;

   Rox_Sint dxm = 0, dxp = 0, dym = 0, dyp = 0, dx = 0, dy = 0;
   Rox_Float theta = 0.0;
   Rox_Uint scale = 0;
   Rox_Sint cols = 0, rows = 0;

   Rox_Uchar ** src = NULL;
   Rox_Uint ** ptrMag = NULL;
   Rox_Uint * ptrTMag = NULL;
   Rox_Float * ptrTheta = NULL;

   Rox_Uint count_filtered_1 = 0;
   Rox_Uint count_filtered_2 = 0;

   // Hard coded thresholds

   Rox_Uint scale_threshold_1 = SQUARE_SCALE_GRADIENT_MIN;
   Rox_Uint scale_threshold_2 = SQUARE_SCALE_GRADIENT_MAX;


   if (!Imagval || !Imag || !Itheta || !image_gray || !mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&rows, &cols, image_gray); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_data_pointer_to_pointer(&src, image_gray);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer(&ptrMag, Imagval);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ptrTMag = Imag;
   ptrTheta = Itheta;


   if(!src || !ptrMag || !ptrTMag || !ptrTheta) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Ignore the borders
   for ( Rox_Sint i = 1; i < rows - 1; i++)
   {
      Rox_Uchar * ptrprpc = &src[i - 1][0];
      Rox_Uchar * ptrprcc = &src[i - 1][1];
      Rox_Uchar * ptrprnc = &src[i - 1][2];
      Rox_Uchar * ptrcrpc = &src[  i  ][0];
      Rox_Uchar * ptrcrcc = &src[  i  ][1];
      Rox_Uchar * ptrcrnc = &src[  i  ][2];
      Rox_Uchar * ptrnrpc = &src[i + 1][0];
      Rox_Uchar * ptrnrcc = &src[i + 1][1];
      Rox_Uchar * ptrnrnc = &src[i + 1][2];

      for ( Rox_Sint j = 1; j < cols - 1; j++)
      {
         idx = i * cols + j;

         // Compute Sobel image gradient
         dyp = 2*(*ptrnrcc) + (*ptrnrpc) + (*ptrnrnc);
         dym = 2*(*ptrprcc) + (*ptrprpc) + (*ptrprnc);

         dxp = 2*(*ptrcrnc) + (*ptrprnc) + (*ptrnrnc);
         dxm = 2*(*ptrcrpc) + (*ptrprpc) + (*ptrnrpc);

         dx = dxp - dxm;
         dy = dyp - dym;

         ptrprpc++;
         ptrprcc++;
         ptrprnc++;
         ptrcrpc++;
         ptrcrcc++;
         ptrcrnc++;
         ptrnrpc++;
         ptrnrcc++;
         ptrnrnc++;

         // Compute scale of gradient (scale = square of scale)
         scale = dx*dx + dy*dy;

         if (scale < scale_threshold_1)
         {
            ptrMag[i][j] = 0;
            ptrTheta[idx] = 0;
#ifdef NOFILTER_ISOLATED_PIXELS
            ptrTMag[idx] = 0;
#endif
            count_filtered_1++;
            continue;
         }

         // Compute angle of gradient
         theta = quad_arctan2((Rox_Float) dy, (Rox_Float) dx);

         // Store scale and gradient
         ptrMag[i][j] = scale;
         ptrTheta[idx] = theta;
#ifdef NOFILTER_ISOLATED_PIXELS
         ptrTMag[idx] = scale;
#endif

      }
   }

#ifndef NOFILTER_ISOLATED_PIXELS
   // We keep only gradients that have sufficiently strong neighbors
   // so that we filter isolated points
   // We do not consider the borders ( 2 pixel on each border )
   for ( Rox_Sint i = 2; i < rows - 2; i++)
   {
      for ( Rox_Sint j = 2; j < cols - 2; j++)
      {
         idx = i * cols + j;

         scale = ptrMag[i][j];

         if (scale < scale_threshold_1)
         {
            ptrTMag[idx] = 0;
            continue;
         }
         else // if (scale < scale_threshold_2)
         {
            // Check if at last a neighboor (3x3 box) is strong enough
            scale = ptrMag[i-1][j];
            scale = ROX_MAX(scale, ptrMag[i-1][j-1]);
            scale = ROX_MAX(scale, ptrMag[i-1][j+1]);
            scale = ROX_MAX(scale, ptrMag[i  ][j-1]);
            scale = ROX_MAX(scale, ptrMag[i  ][j+1]);
            scale = ROX_MAX(scale, ptrMag[i+1][j-1]);
            scale = ROX_MAX(scale, ptrMag[i+1][j  ]);
            scale = ROX_MAX(scale, ptrMag[i+1][j+1]);

            if (scale < scale_threshold_2)
            {
               count_filtered_2++;
               ptrTMag[idx] = 0;
               continue;
            }
         }

         ptrTMag[idx] = ptrMag[i][j];
      }
   }
#endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_gradient_angle_scale_nomask_filter_buffers(
   Rox_Array2D_Uint mag_tmp,
   Rox_Float * ptrTheta,
   Rox_Uint * ptrTMag,
   Rox_Uint scale_threshold_2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0;
   Rox_Uint ** ptrMag_tmp = NULL;
   Rox_Uint count_filtered_2 = 0;


   error = rox_array2d_uint_get_size(&rows, &cols, mag_tmp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_set_data ( mag_tmp, ptrTMag, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer(&ptrMag_tmp, mag_tmp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We do not consider the borders ( 2 pixels on each border )

// #ifdef ROX_USES_OPENMP
//    #pragma omp parallel for schedule(dynamic)
// #endif
   for ( Rox_Sint i = 2; i < rows - 2; i++)
   {
      for ( Rox_Sint j = 2; j < cols - 2; j++)
      {
         Rox_Uint idx = i * cols + j;

         Rox_Uint scale = ptrMag_tmp[i][j];

         if (scale > 0)
         {
            // Check if at last a neighboor (3x3 box) is strong enough
            scale = ptrMag_tmp[i-1][j];
            scale = ROX_MAX(scale, ptrMag_tmp[i-1][j-1]);
            scale = ROX_MAX(scale, ptrMag_tmp[i-1][j+1]);
            scale = ROX_MAX(scale, ptrMag_tmp[i  ][j-1]);
            scale = ROX_MAX(scale, ptrMag_tmp[i  ][j+1]);
            scale = ROX_MAX(scale, ptrMag_tmp[i+1][j-1]);
            scale = ROX_MAX(scale, ptrMag_tmp[i+1][j  ]);
            scale = ROX_MAX(scale, ptrMag_tmp[i+1][j+1]);

            if (scale < scale_threshold_2)
            {
               ptrTMag[idx] = 0;
               ptrTheta[idx] = 0;
               count_filtered_2++;
               continue;
            }
         }

         ptrTMag[idx] = ptrMag_tmp[i][j];
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_gradientclusterer_buildgradients_refactor ( // refactor
   Rox_Array2D_Uint mag_tmp,
   Rox_Uint * Imag,
   Rox_Float * Itheta,
   Rox_Image image_gray,
   Rox_Imask imask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0;

   // Hard coded thresholds
   Rox_Uint scale_threshold_1 = SQUARE_SCALE_GRADIENT_MIN;
   Rox_Uint scale_threshold_2 = SQUARE_SCALE_GRADIENT_MAX;

   if(!mag_tmp || !Imag || !Itheta || !image_gray || !imask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&rows, &cols, image_gray); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_gradient_sobel_angle_scale_nomask_buffers ( Itheta, Imag, image_gray, scale_threshold_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifndef NOFILTER_ISOLATED_PIXELS
   // We keep only gradients that have sufficiently strong neighbors
   // so that we filter isolated points
   error = rox_image_gradient_angle_scale_nomask_filter_buffers ( mag_tmp, Itheta, Imag, scale_threshold_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );
#endif

function_terminate:

   return error;
}