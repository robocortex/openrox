//==============================================================================
//
//    OPENROX   : File meanvar.c
//
//    Contents  : Implementation of meanvar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_meanvar.h"
#include <inout/system/errors_print.h>

int rox_ansi_array_float_meanvar ( float * mean, float * variance, float * input_data, int size )
{
   int error = 0;

   float sum = 0.0, lmean = 0.0, delta = 0.0;

   for ( int i = 0; i < size; i++)
   {       
      sum += input_data[i];
   }

   lmean = sum / (float) size;
   *mean = lmean;

   sum = 0.0;
   for ( int i = 0; i < size; i++)
   {
      delta = input_data[i] - lmean;
      sum += (delta * delta);
   }

   lmean = sum / (float) size;
   *variance = lmean;

   return error;
}


int rox_ansi_array_float_meanvar_mask ( float * mean, float * variance, float * input_data, unsigned int * mask_data, int size )
{
   int error = 0;

   float sum = 0.0, lmean = 0.0, delta = 0.0;
   size_t count = 0;

   for ( int i = 0; i < size; i++)
   {
      if (!mask_data[i]) continue;
       
      sum += input_data[i];
      count++;
   }

   if (count == 0) *mean = 0;
   else
   {
      lmean = sum / (float) count;
      *mean = lmean;
   }

   sum = 0.0;
   for ( int i = 0; i < size; i++)
   {
      if (!mask_data[i]) continue;

      delta = input_data[i] - lmean;
      sum += (delta * delta);
   }

   lmean = sum / (float)(count-1);
   *variance = lmean;

   return error;
}

