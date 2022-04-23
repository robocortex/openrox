//==============================================================================
//
//    OPENROX   : File ansi_mulmatmat.c
//
//    Contents  : Implementation of ansi mulmatmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_mulmatmat.h"

int rox_ansi_array_float_mulmatmat ( float * res_data, int res_rows, int res_cols, float * one_data, float * two_data, int two_rows )
{
   int error = 0;

   // NB one_rows = res_rows
   int one_cols = two_rows;
   int two_cols = res_cols;
   for ( int i = 0; i < res_rows; i++)
   {
      for ( int j = 0; j < res_cols; j++)
      {
         res_data[i*res_cols+j] = 0;

         for ( int k = 0; k < two_rows; k++)
         {
            res_data[i*res_cols+j] += one_data[i*one_cols+k] * two_data[k*two_cols+j];
         }
      }
   }

   return error;
}


int rox_ansi_array_double_mulmatmat ( double * res_data, int res_rows, int res_cols, double * one_data, double * two_data, int two_rows )
{
   int error = 0;

   // NB one_rows = res_rows
   int one_cols = two_rows;
   int two_cols = res_cols;
   for ( int i = 0; i < res_rows; i++)
   {
      for ( int j = 0; j < res_cols; j++)
      {
         res_data[i*res_cols+j] = 0;

         for ( int k = 0; k < two_rows; k++)
         {
            res_data[i*res_cols+j] += one_data[i*one_cols+k] * two_data[k*two_cols+j];
         }
      }
   }

   return error;
}


int rox_ansi_array2d_double_mulmatmat ( double ** res_data, int res_rows, int res_cols, double ** one_data, double ** two_data, int two_rows )
{
   int error = 0;

   for ( int i = 0; i < res_rows; i++)
   {
      for ( int j = 0; j < res_cols; j++)
      {
         res_data[i][j] = 0;

         for ( int k = 0; k < two_rows; k++)
         {
            res_data[i][j] += one_data[i][k] * two_data[k][j];
         }
      }
   }

   return error;
}
