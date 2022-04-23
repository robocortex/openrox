//==============================================================================
//
//    OPENROX   : File ansi_mulmattransmat.c
//
//    Contents  : Implementation of ansi mulmattransmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_mulmattransmat.h"

// one_data is a row-wise array containing a q x n matrix
// two_data is a row-wise array containing a q x m matrix
// res_data is a row-wise array containing a n x m matrix
// compute res_data = transpose (one_data) * two_data
// res_rows = n
// res_cols = m
// two_rows = q
int rox_ansi_array_float_mulmattransmat ( float * res_data, int res_rows, int res_cols, float * one_data, float * two_data, int two_rows )
{
   int error = 0;

   // NB one_rows = two_rows
   int one_cols = res_rows;
   int two_cols = res_cols;
   for ( int i = 0; i < res_rows; i++)
   {
      for ( int j = 0; j < res_cols; j++)
      {
         res_data[i*res_cols+j] = 0;

         for ( int k = 0; k < two_rows; k++)
         {
            res_data[i*res_cols+j] += one_data[k*one_cols+i] * two_data[k*two_cols+j];
         }
      }
   }

   return error;
}

// one_data is a row-wise array containing a q x n matrix
// two_data is a row-wise array containing a q x m matrix
// res_data is a row-wise array containing a n x m matrix
// compute res_data = transpose (one_data) * two_data
// res_rows = n
// res_cols = m
// two_rows = q
int rox_ansi_array_double_mulmattransmat ( double * res_data, int res_rows, int res_cols, double * one_data, double * two_data, int two_rows )
{
   int error = 0;

   // NB one_rows = two_rows
   int one_cols = res_rows;
   int two_cols = res_cols;
   for ( int i = 0; i < res_rows; i++)
   {
      for ( int j = 0; j < res_cols; j++)
      {
         res_data[i*res_cols+j] = 0;

         for ( int k = 0; k < two_rows; k++)
         {
            res_data[i*res_cols+j] += one_data[k*one_cols+i] * two_data[k*two_cols+j];
         }
      }
   }

   return error;
}

int rox_ansi_array2d_double_mulmattransmat ( double ** res_data, int res_rows, int res_cols, double ** one_data, double ** two_data, int two_rows )
{
   int error = 0;

   for ( int i = 0; i < res_rows; i++)
   {
      for ( int j = 0; j < res_cols; j++)
      {
         res_data[i][j] = 0;

         for ( int k = 0; k < two_rows; k++)
         {
            res_data[i][j] += one_data[k][i] * two_data[k][j];
         }
      }
   }

   return error;
}
