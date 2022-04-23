//============================================================================
//
//    OPENROX   : File array2d_uchar_symmetric_separable_convolve.c
//
//    Contents  : Implementation of array2d_uchar_symmetric_separable_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "array2d_uchar_symmetric_separable_convolve.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>
#include <string.h>

// #define SYMM_IDX(P,MAX) ((P) < 0)?-(P):(((P) > (MAX))?(MAX)-((P)-(MAX)):(P))
// #define SYMM_IDX_MIN(P) (((P) < 0)?-(P):(P))
// #define SYMM_IDX_MAX(P,MAX) (((P) > (MAX))?(MAX)-((P)-(MAX)):(P))

Rox_ErrorCode rox_array2d_uchar_symmetric_seperable_convolve (
   Rox_Image output, 
   const Rox_Image input, 
   const Rox_Array2D_Float kernel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Float buffer = NULL;
   Rox_Image imborder = NULL;

   if ( !output )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !input || !kernel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint ksize = 0;
   error = rox_array2d_float_get_cols(&ksize, kernel);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hksize = ksize / 2;
   if (ksize % 2 == 0) 
   { error = ROX_ERROR_VALUE_NOT_ODD; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_uchar_check_size ( input, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &buffer, cols, rows + 2 * hksize );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_new ( &imborder, rows, cols + 2 * hksize );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get kernel pointer
   Rox_Float ** ddk = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &ddk, kernel );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float * dk = &ddk[0][hksize];

   Rox_Uchar ** din = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &din, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dout = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dout, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dib = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dib, imborder );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** db = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &db, buffer );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Create image with borders mirrored
   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Uchar * row_ib = &dib[i][hksize];
      Rox_Uchar * row_in = din[i];

      memcpy(row_ib, row_in, sizeof(Rox_Uchar) * cols);

      for (Rox_Sint j = 1; j <= hksize; j++) row_ib[-j] = row_in[0];
         
      for (Rox_Sint j = cols; j < cols + hksize; j++) row_ib[j] = row_in[cols - 1];
   }

   // Horizontal
   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Uchar * row_ib = &dib[i][hksize];

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Float val = ((Rox_Float) row_ib[j]) * dk[0];

         for (Rox_Sint k = 1; k <= hksize; k++)
         {
            val += (((Rox_Float) row_ib[j + k]) + ((Rox_Float) row_ib[j - k])) * dk[k];
         }

         db[j][i + hksize] = val;
      }
   }

   for (Rox_Sint i = 0; i < cols; i++)
   {
      Rox_Float * row_b = &db[i][hksize];

      for (Rox_Sint j = 1; j <= hksize; j++) row_b[-j] = row_b[0];
      for (Rox_Sint j = rows; j < rows + hksize; j++) row_b[j] = row_b[rows - 1];
   }

   // Vertical transposed
   for (Rox_Sint i = cols - 1; i >= 0; i--)
   {
      Rox_Float * row_b = &db[i][hksize];

      for (Rox_Sint j = 0; j < rows; j++)
      {
         Rox_Float val = row_b[j] * dk[0];

         for (Rox_Sint k = 1; k <= hksize; k++)
         {
            val += (row_b[j - k] + row_b[j + k]) * dk[k];
         }

         // Store transpose transposed ;)
         dout[j][i] = (Rox_Uchar)(val + 0.5);
      }
   }

function_terminate:
   rox_array2d_float_del(&buffer);
   rox_array2d_uchar_del(&imborder);

   return error;
}
