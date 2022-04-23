//============================================================================
//
//    OPENROX   : File array2d_float_symmetric_separable_convolve_sse.c
//
//    Contents  : Implementation of array2d_float_symmetric_separable_convolve module with SSE optimisation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "array2d_float_symmetric_separable_convolve.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>
#include <string.h>

int rox_ansi_array2d_float_symmetric_seperable_convolve_sse ( 
   float ** output_data,
   float ** input_data,
   int rows,
   int cols,
   int stride,
   float ** kernel_data,
   int hksize
   //float ** buffer_data,
   //float ** imborder_data
)
{
   int error = 0;

   int stephor = stride / sizeof(Rox_Float);
   int ssesizehor = stephor / 4;
   int ssesizever = (rows / 4);
   int border_nosse = rows - (ssesizever * 4);

   float * dk = &kernel_data[0][hksize];

   float ** buffer_data = (float**)malloc(ssesizehor * 4 * sizeof(float*));
   for (int i = 0; i < ssesizehor * 4; i++)
       buffer_data[i] = (float*)malloc( (rows + 2 * hksize) * sizeof(float));

   float ** imborder_data = (float**)malloc(rows * sizeof(float*));
   for (int i = 0; i < rows; i++)
       imborder_data[i] = (float*)malloc( (cols + 2 * hksize) * sizeof(float));


   // Create image with borders mirrored
   for (int i = 0; i < rows; i++)
   {
      float * row_b = &imborder_data[i][hksize];
      float * row_in = input_data[i];
      memcpy(row_b, row_in, sizeof(float) * cols);
      for (int j = 1; j <= hksize; j++) row_b[-j] = row_in[0];
      for (int j = cols; j < cols + hksize; j++) row_b[j] = row_in[cols-1];
   }
   
   __m128 vinl, vinr;
   __m128 vk;
   __m128 vadd;
   union ssevector vval;

   for (int i = 0; i < rows; i++)
   {
      float * row_in = &imborder_data[i][hksize];

      for (int j = 0; j < ssesizehor; j++)
      {
         int j4 = j * 4;
         vinl = _mm_loadu_ps(row_in);
         vk = _mm_load1_ps(&dk[0]);
         vval.sse = _mm_mul_ps(vinl, vk);

         for (int k = 1; k <= hksize; k++)
         {
            vinr = _mm_loadu_ps(&row_in[k]);
            vinl = _mm_loadu_ps(&row_in[-k]);
            vk = _mm_load1_ps(&dk[k]);
            vadd = _mm_add_ps(vinl, vinr);
            vadd = _mm_mul_ps(vadd, vk);
            vval.sse = _mm_add_ps(vval.sse, vadd);
         }

         buffer_data[j4][i + hksize] = vval.tab[0];
         buffer_data[j4+1][i + hksize] = vval.tab[1];
         buffer_data[j4+2][i + hksize] = vval.tab[2];
         buffer_data[j4+3][i + hksize] = vval.tab[3];

         row_in += 4;
      }
   }

   for (int i = 0; i < cols; i++)
   {
      float * row_b = &buffer_data[i][hksize];

      for (int j = 1; j <= hksize; j++) row_b[-j] = row_b[0];
      for (int j = rows; j < rows + hksize; j++) row_b[j] = row_b[rows-1];
   }

   for (int i = 0; i < cols; i++)
   {
      float * row_in = &buffer_data[i][hksize];

      int j4 = 0;

      for (int j = 0; j < ssesizever; j++)
      {
         j4 = j * 4;
         vinl = _mm_loadu_ps(row_in);
         vk = _mm_load1_ps(&dk[0]);
         vval.sse = _mm_mul_ps(vinl, vk);

         for (int k = 1; k <= hksize; k++)
         {
            vinr = _mm_loadu_ps(&row_in[k]);
            vk = _mm_load1_ps(&dk[k]);
            vinl = _mm_loadu_ps(&row_in[-k]);
            vadd = _mm_add_ps(vinl, vinr);
            vadd = _mm_mul_ps(vadd, vk);
            vval.sse = _mm_add_ps(vval.sse, vadd);
         }

         output_data[j4+0][i] = vval.tab[0];
         output_data[j4+1][i] = vval.tab[1];
         output_data[j4+2][i] = vval.tab[2];
         output_data[j4+3][i] = vval.tab[3];

         row_in += 4;
      }

      j4 += 4;

      //Continue without sse for rows beyond multiple of 4
      for (int j = 0; j < border_nosse; j++)
      {
         float val = row_in[j] * dk[0];

         for (int k = 1; k <= hksize; k++)
         {
            val += (row_in[j-k] + row_in[j+k]) * dk[k];
         }

         output_data[j4+j][i] = val;
      }
   }

   return error;
}

Rox_ErrorCode rox_array2d_float_symmetric_seperable_convolve ( 
   Rox_Array2D_Float output,
   Rox_Array2D_Float input, 
   Rox_Array2D_Float kernel 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Float buffer = NULL;
   Rox_Array2D_Float imborder = NULL;

   if ( !output || !input || !kernel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint ksize = 0;
   error = rox_array2d_float_get_cols(&ksize, kernel);
   
   if (ksize % 2 == 0)
   { error = ROX_ERROR_VALUE_NOT_ODD; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_check_size ( input, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hksize = ksize / 2;

   Rox_Sint stride = 0;
   error = rox_array2d_float_get_stride ( &stride, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stephor = stride / sizeof(Rox_Float);
   Rox_Sint ssesizehor = stephor / 4;

   error = rox_array2d_float_new(&buffer, ssesizehor * 4 , rows + 2 * hksize); // 4 = SSE rows
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&imborder, rows, cols + 2 * hksize );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get kernel pointer
   Rox_Float ** kernel_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &kernel_data, kernel );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** input_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &input_data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** output_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &output_data, output );
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Float ** buffer_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &buffer_data, buffer );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** imborder_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &imborder_data, imborder );
   ROX_ERROR_CHECK_TERMINATE ( error );

//#define integrated
#ifdef integrated

   Rox_Sint ssesizever = (rows / 4);
   Rox_Sint border_nosse = rows - (ssesizever * 4);

   Rox_Float * dk = &kernel_data[0][hksize];

   // Create image with borders mirrored
   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Float * row_b = &imborder_data[i][hksize];
      Rox_Float * row_in = input_data[i];
      memcpy(row_b, row_in, sizeof(Rox_Float) * cols);
      for (Rox_Sint j = 1; j <= hksize; j++) row_b[-j] = row_in[0];
      for (Rox_Sint j = cols; j < cols + hksize; j++) row_b[j] = row_in[cols-1];
   }
   
   __m128 vinl, vinr;
   __m128 vk;
   __m128 vadd;
   union ssevector vval;

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Float * row_in = &imborder_data[i][hksize];

      for (Rox_Sint j = 0; j < ssesizehor; j++)
      {
         Rox_Sint j4 = j * 4;
         vinl = _mm_loadu_ps(row_in);
         vk = _mm_load1_ps(&dk[0]);
         vval.sse = _mm_mul_ps(vinl, vk);

         for (Rox_Sint k = 1; k <= hksize; k++)
         {
            vinr = _mm_loadu_ps(&row_in[k]);
            vinl = _mm_loadu_ps(&row_in[-k]);
            vk = _mm_load1_ps(&dk[k]);
            vadd = _mm_add_ps(vinl, vinr);
            vadd = _mm_mul_ps(vadd, vk);
            vval.sse = _mm_add_ps(vval.sse, vadd);
         }

         buffer_data[j4][i + hksize] = vval.tab[0];
         buffer_data[j4+1][i + hksize] = vval.tab[1];
         buffer_data[j4+2][i + hksize] = vval.tab[2];
         buffer_data[j4+3][i + hksize] = vval.tab[3];

         row_in += 4;
      }
   }

   for (Rox_Sint i = 0; i < cols; i++)
   {
      Rox_Float * row_b = &buffer_data[i][hksize];

      for (Rox_Sint j = 1; j <= hksize; j++) row_b[-j] = row_b[0];
      for (Rox_Sint j = rows; j < rows + hksize; j++) row_b[j] = row_b[rows-1];
   }

   for (Rox_Sint i = 0; i < cols; i++)
   {
      Rox_Float * row_in = &buffer_data[i][hksize];

      Rox_Sint j4 = 0;

      for (Rox_Sint j = 0; j < ssesizever; j++)
      {
         j4 = j * 4;
         vinl = _mm_loadu_ps(row_in);
         vk = _mm_load1_ps(&dk[0]);
         vval.sse = _mm_mul_ps(vinl, vk);

         for (Rox_Sint k = 1; k <= hksize; k++)
         {
            vinr = _mm_loadu_ps(&row_in[k]);
            vk = _mm_load1_ps(&dk[k]);
            vinl = _mm_loadu_ps(&row_in[-k]);
            vadd = _mm_add_ps(vinl, vinr);
            vadd = _mm_mul_ps(vadd, vk);
            vval.sse = _mm_add_ps(vval.sse, vadd);
         }

         output_data[j4+0][i] = vval.tab[0];
         output_data[j4+1][i] = vval.tab[1];
         output_data[j4+2][i] = vval.tab[2];
         output_data[j4+3][i] = vval.tab[3];

         row_in += 4;
      }

      j4 += 4;

      //Continue without sse for rows beyond multiple of 4
      for (Rox_Sint j = 0; j < border_nosse; j++)
      {
         Rox_Float val = row_in[j] * dk[0];

         for (Rox_Sint k = 1; k <= hksize; k++)
         {
            val += (row_in[j-k] + row_in[j+k]) * dk[k];
         }

         output_data[j4+j][i] = val;
      }
   }
#else

error = rox_ansi_array2d_float_symmetric_seperable_convolve_sse ( 
            output_data,
            input_data,
            rows,
            cols,
            stride,
            kernel_data,
            hksize//,
            //buffer_data,
            //imborder_data
);

#endif

   error = ROX_ERROR_NONE;

function_terminate:
   rox_array2d_float_del(&buffer);
   rox_array2d_float_del(&imborder);

   return error;
}