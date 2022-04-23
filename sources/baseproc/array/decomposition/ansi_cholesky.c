//==============================================================================
//
//    OPENROX   : File ansi_cholesky.c
//
//    Contents  : Implementation of ansi cholesky module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_cholesky.h"
#include <math.h>

int rox_ansi_array_float_cholesky_decomposition_inplace ( float * L_data, int n )
{   
   int error = 0;

   for ( int i = 0; i < n; i++)
   {
      for ( int  j = i; j < n; j++)
      {
         float sum = L_data[i*n+j];

         for ( int k = i - 1; k>= 0; k--)
         {
            sum -= L_data[i*n+k] * L_data[j*n+k];
         }

         if (i == j)
         {
            if (sum <= 0.0f) 
            { error = 6; goto function_terminate;}

            L_data[i*n+i] = sqrtf(sum);
         }
         else
         {
            L_data[j*n+i] = sum / L_data[i*n+i];
         }
      }
   }

   // L must be a Lower triangular matrix : set to 0 the entries of the upper part of the matrix 
   for ( int i = 0; i < n; i++)
   {
      for ( int j = 0; j < i; j++)
      {
         L_data[j*n+i] = 0.0f;
      }
   }

function_terminate:
   return error;
}

int rox_ansi_array_double_cholesky_decomposition_inplace ( double * L_data, int n )
{   
   int error = 0;

   for ( int i = 0; i < n; i++)
   {
      for ( int  j = i; j < n; j++)
      {
         double sum = L_data[i*n+j];

         for ( int k = i - 1; k>= 0; k--)
         {
            sum -= L_data[i*n+k] * L_data[j*n+k];
         }

         if (i == j)
         {
            if (sum <= 0.0) 
            { error = -1; goto function_terminate; }

            L_data[i*n+i] = sqrt(sum);
         }
         else
         {
            L_data[j*n+i] = sum / L_data[i*n+i];
         }
      }
   }

   // L must be a Lower triangular matrix : set to 0 the entries of the upper part of the matrix 
   for ( int i = 0; i < n; i++)
   {
      for ( int j = 0; j < i; j++)
      {
         L_data[j*n+i] = 0.0;
      }
   }

function_terminate:
   return error;
}
