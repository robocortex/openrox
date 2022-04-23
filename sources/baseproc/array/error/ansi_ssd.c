//==============================================================================
//
//    OPENROX   : File ansi_ssd.c
//
//    Contents  : Implementation of ansi_ssd module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_ssd.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

int rox_ansi_array2d_float_ss_mask (
   double * ss, 
   float ** array2d_data,
   const int rows,
   const int cols, 
   unsigned int ** mask_data
)
{
   int error = 0;
   double ss_tmp = 0.0;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         if (mask_data[r][c] == 0) continue;

         ss_tmp += array2d_data[r][c] * array2d_data[r][c];
      }
   }

   *ss = ss_tmp;

   return error;
}

int rox_ansi_array2d_double_ssd (
   double * ssd, 
   double ** array2d_1_data, 
   double ** array2d_2_data,
   const int rows,
   const int cols
)
{
   int error = 0;
   
   double ssd_tmp = 0.0;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         double dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         ssd_tmp += dif*dif;
      }
   }

   *ssd = ssd_tmp;

   return error;
}

int rox_ansi_array2d_float_ssd (
   float * ssd, 
   float ** array2d_1_data, 
   float ** array2d_2_data,
   const int rows,
   const int cols
)
{
   int error = 0;
   float ssd_tmp = 0.0f;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         float dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         ssd_tmp += dif*dif;
      }
   }

   *ssd = ssd_tmp;

   return error;
}

int rox_ansi_array2d_uchar_ssd (
   float * ssd, 
   unsigned char ** array2d_1_data, 
   unsigned char ** array2d_2_data,
   const int rows,
   const int cols
)
{
   int error = 0;
   float ssd_tmp = 0.0f;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         float dif = (float) array2d_2_data[r][c] - (float) array2d_1_data[r][c];

         ssd_tmp += dif*dif;
      }
   }

   *ssd = ssd_tmp;

   return error;
}

int rox_ansi_array2d_double_ssd_mask (
   double * ssd, 
   double ** array2d_1_data, 
   double ** array2d_2_data,
   const int rows,
   const int cols,
   unsigned int ** mask_data
)
{
   int error = 0;

   double ssd_tmp = 0.0;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         double dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            ssd_tmp += dif*dif;
         }
      }
   }

   *ssd = ssd_tmp;

   return error;
}

int rox_ansi_array2d_float_ssd_mask (
   float * ssd, 
   float ** array2d_1_data, 
   float ** array2d_2_data,
   const int rows,
   const int cols,
   unsigned int ** mask_data
)
{
   int error = 0;

   float ssd_tmp = 0.0f;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         float dif = array2d_2_data[r][c] - array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            ssd_tmp += dif*dif;
         }
      }
   }

   *ssd = ssd_tmp;

   return error;
}

int rox_ansi_array2d_uchar_ssd_mask (
   float * ssd, 
   unsigned char ** array2d_1_data, 
   unsigned char ** array2d_2_data,
   const int rows,
   const int cols,
   unsigned int ** mask_data
)
{
   int error = 0;

   float ssd_tmp = 0.0f;
   for ( int r = 0; r < rows; r++)
   {
      for ( int c = 0; c < cols; c++)
      {
         float dif = (float) array2d_2_data[r][c] - (float) array2d_1_data[r][c];

         if (mask_data[r][c])
         {
            ssd_tmp += dif*dif;
         }
      }
   }

   *ssd = ssd_tmp;

   return error;
}
