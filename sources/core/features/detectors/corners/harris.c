//==============================================================================
//
//    OPENROX   : File harris.c
//
//    Contents  : Implementation of harris module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "harris.h"

#include <float.h>

#include <generated/array2d_sint.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/integral/integralsum.h>
#include <baseproc/array/integral/integralaccess.h>
#include <baseproc/array/morphological/dilate_grayone.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_harris_detector_uchar(Rox_Array2D_Float response, Rox_Image source, Rox_Uint radius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar ** dsrc;
   Rox_Float ** dres;
   Rox_Sint ** dgxx, ** dgxy, ** dgyy;
   Rox_Slint ** dintgxx, ** dintgxy, ** dintgyy;
   Rox_Array2D_Sint igxx = NULL, igxy = NULL, igyy = NULL;
   Rox_Array2D_Slint intgxx = NULL, intgxy = NULL, intgyy = NULL;
   Rox_Slint lxx, lxy, lyy, det, trace;
   Rox_Float score;

   Rox_Sshort pc, nc, pr, nr, gx, gy;
   Rox_Float gxx, gyy, gxy;

   Rox_Uchar * prevrow = NULL, * nextrow = NULL, * currow = NULL;

   if (!response || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   Rox_Sint rows = 0, cols = 0; 
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(response, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&igxx, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&igxy, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_new(&igyy, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_slint_new(&intgxx, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_slint_new(&intgxy, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_slint_new(&intgyy, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer(&dres, response);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&dsrc, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_get_data_pointer_to_pointer(&dgxx, igxx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_get_data_pointer_to_pointer(&dgyy, igyy);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sint_get_data_pointer_to_pointer(&dgxy, igxy);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_slint_get_data_pointer_to_pointer( &dintgxx, intgxx);
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_slint_get_data_pointer_to_pointer( &dintgyy, intgyy);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_slint_get_data_pointer_to_pointer( &dintgxy, intgxy);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      dgxx[i][0] = 0;
      dgyy[i][0] = 0;
      dgxy[i][0] = 0;
      dgxx[i][cols - 1] = 0;
      dgyy[i][cols - 1] = 0;
      dgxy[i][cols - 1] = 0;
      dres[i][0] = 0;
      dres[i][cols - 1] = 0;
   }

   for (Rox_Sint j = 0; j < cols; j++)
   {
      dgxx[0][j] = 0;
      dgyy[0][j] = 0;
      dgxy[0][j] = 0;
      dgxx[rows - 1][j] = 0;
      dgyy[rows - 1][j] = 0;
      dgxy[rows - 1][j] = 0;
      dres[0][j] = 0;
      dres[rows - 1][j] = 0;
   }

   // Compute GxGx, GxGy, GyGy for image
   for (Rox_Sint i = 1; i < rows - 1; i++)
   {
      nextrow = dsrc[(int)((int)i + (int)1)];
      prevrow = dsrc[i - 1];
      currow = dsrc[i];

      for (Rox_Sint j = 1; j < cols - 1; j++)
      {
         pr = prevrow[j];
         nr = nextrow[j];
         pc = currow[j - 1];
         nc = currow[j + 1];

         gx = nc - pc;
         gy = nr - pr;

         gxx = (Rox_Float) (gx * gx);
         gyy = (Rox_Float) (gy * gy);
         gxy = (Rox_Float) (gx * gy);

         dgxx[i][j] = (Rox_Sint) gxx;
         dgyy[i][j] = (Rox_Sint) gyy;
         dgxy[i][j] = (Rox_Sint) gxy;
      }
   }

   // Compute correlation matrices
   error = rox_array2d_sint_integralsum(intgxx, igxx); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_sint_integralsum(intgyy, igyy); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_sint_integralsum(intgxy, igxy); ROX_ERROR_CHECK_TERMINATE(error)

   // Compute harris score
   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Sint top, bottom, sizeh;

      top = i - radius;
      bottom = i + radius;
      if (top < 0) top = 0;
      if (bottom > rows - 1) bottom = rows - 1;
      sizeh = bottom - top + 1;

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Sint left, right, sizew;

         left = j - radius;
         right = j + radius;
         if (left < 0) left = 0;
         if (right > cols - 1) right = cols - 1;
         sizew = right - left + 1;

         lxx = rox_array2d_slint_integralaccess(dintgxx, left, top, sizew, sizeh);
         lyy = rox_array2d_slint_integralaccess(dintgyy, left, top, sizew, sizeh);
         lxy = rox_array2d_slint_integralaccess(dintgxy, left, top, sizew, sizeh);
         det = lxx * lyy - lxy * lxy;
         trace = lxx + lyy;

         score = ((float)det) - 0.04f * (float)(trace * trace);
         dres[i][j] = score;
      }
   }

function_terminate:
   rox_array2d_sint_del(&igxx);
   rox_array2d_sint_del(&igxy);
   rox_array2d_sint_del(&igyy);
   rox_array2d_slint_del(&intgxx);
   rox_array2d_slint_del(&intgxy);
   rox_array2d_slint_del(&intgyy);

   return error;
}
