//==============================================================================
//
//    OPENROX   : File array2d_uchar_zncc.c
//
//    Contents  : Implementation of array2d_uchar_zncc module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_uchar_zncc.h"

#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>

int rox_ansi_array2d_uchar_zncc (
   double * zncc,
   unsigned char ** one_data,
   unsigned char ** two_data,
   unsigned int  ** mask_data,
   int rows,
   int cols
)
{
   int error = 0;
   int count = 0;
   
   long int gcc = 0;
   long int sum1 = 0, sum2 = 0, sumsq1 = 0, sumsq2 = 0;
   
   for ( int i = 0; i < rows; i++)
   {
      for ( int j = 0; j < cols; j++)
      {
         if ( !mask_data[i][j] ) continue;

         sum1 += one_data[i][j];
         sum2 += two_data[i][j];
         sumsq1 += one_data[i][j] * one_data[i][j];
         sumsq2 += two_data[i][j] * two_data[i][j];
         gcc += one_data[i][j] * two_data[i][j];
         count++;
      }
   }

   if (count == 0) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   double mean1 = ((double)sum1) / ((double)count);
   double mean2 = ((double)sum2) / ((double)count);

   double nom = ((double)gcc) - mean1*mean2*(double)count;
   double denom1 = (double)sumsq1 - mean1 * mean1 * (double)count;
   double denom2 = (double)sumsq2 - mean2 * mean2 * (double)count;

   // if (denom1 == 0.0 || denom2 == 0.0) {error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE(error)}

   if (denom1 < DBL_EPSILON || denom2 < DBL_EPSILON) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *zncc = nom / (sqrt(denom1)*sqrt(denom2));

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_zncc (
   Rox_Double * zncc,
   const Rox_Array2D_Uchar one,
   const Rox_Array2D_Uchar two,
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Slint gcc = 0;
   Rox_Slint sum1 = 0, sum2 = 0, sumsq1 = 0, sumsq2 = 0;

   if ( !zncc ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !one || !two || !mask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *zncc = 0.0;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, one); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(two, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** one_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &one_data, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** two_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &two_data, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint count = 0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if ( !mask_data[i][j] ) continue;

         sum1 += one_data[i][j];
         sum2 += two_data[i][j];
         sumsq1 += one_data[i][j] * one_data[i][j];
         sumsq2 += two_data[i][j] * two_data[i][j];
         gcc += one_data[i][j] * two_data[i][j];
         count++;
      }
   }

   if (count == 0) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double mean1 = ((double)sum1) / ((double)count);
   Rox_Double mean2 = ((double)sum2) / ((double)count);

   Rox_Double nom = ((Rox_Double)gcc) - mean1*mean2*(Rox_Double)count;
   Rox_Double denom1 = (Rox_Double)sumsq1 - mean1 * mean1 * (Rox_Double)count;
   Rox_Double denom2 = (Rox_Double)sumsq2 - mean2 * mean2 * (Rox_Double)count;

   // if (denom1 == 0.0 || denom2 == 0.0) {error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE(error)}

   if (denom1 < DBL_EPSILON || denom2 < DBL_EPSILON) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double score = nom / (sqrt(denom1)*sqrt(denom2));
   *zncc = score;

function_terminate:
   return error;
}
