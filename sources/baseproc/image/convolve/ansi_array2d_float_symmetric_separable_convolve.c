//============================================================================
//
//    OPENROX   : File ansi_array2d_float_symmetric_separable_convolve.c
//
//    Contents  : Implementation of array2d_float_symmetric_separable_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_array2d_float_symmetric_separable_convolve.h"
#include <string.h>

int rox_ansi_array2d_float_symmetric_seperable_convolve ( 
   float ** output_data,
   float ** input_data,
   int rows,
   int cols,
   float ** kernel_data,
   int hksize,
   float ** db,
   float ** dib
)
{
   int error = 0;

   float * dk = &kernel_data[0][hksize];

   // Create image with borders mirrored
   for (int i = 0; i < rows; i++)
   {
      // Pointer to the first pixel of the output i row
      float *row_b = &dib[i][hksize];

      // Pointer to the input i row
      float *row_in = input_data[i];

      memcpy(row_b, row_in, sizeof(float) * cols);

      // Replicate pixel to the left
      for ( int j = 1; j <= hksize; j++ ) row_b[-j] = row_in[0];

      // Replicate pixel to the right
      for ( int j = cols; j < cols + hksize; j++ ) row_b[j] = row_in[cols - 1];
   }

   // Horizontal kernel convolution
   for ( int i = 0; i < rows; i++ )
   {
      float *row_in = &dib[i][hksize];

      for ( int j = 0; j < cols; j++ )
      {
         // Convolve pixel
         float val = row_in[j] * dk[0];
         for ( int k = 1; k <= hksize; k++ )
         {
            val += (row_in[j + k] + row_in[j - k]) * dk[k];
         }

         // Store the buffer in a transposed coordinate
         db[j][i + hksize] = val;
      }
   }

   // Generate borders of the intermediate buffer
   for ( int i = 0; i < cols; i++ )
   {
      float * row_b = &db[i][hksize];

      // Replicate to the left
      for ( int j = 1; j <= hksize; j++ ) row_b[-j] = row_b[0];

      // Replicate to the right
      for ( int j = rows; j < rows + hksize; j++ ) row_b[j] = row_b[rows - 1];
   }

   // Vertical convolution
   for (int i = 0; i < cols; i++)
   {
      float *row_in = &db[i][hksize];

      for (int j = 0; j < rows; j++)
      {
         float val = row_in[j] * dk[0];

         for (int k = 1; k <= hksize; k++)
         {
            val += (row_in[j - k] + row_in[j + k]) * dk[k];
         }

         // Retranspose result
         output_data[j][i] = val;
      }
   }

   return error;
}
