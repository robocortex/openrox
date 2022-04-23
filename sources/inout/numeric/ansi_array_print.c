//==============================================================================
//
//    OPENROX   : File array_print.c
//
//    Contents  : Implementation of array print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_array_print.h"

#include <stdio.h>
#include <system/errors/errors.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

int rox_ansi_array_double_print ( double * input, size_t size )
{
   int error = 0;

   rox_log("Array (%lu) : \r\n", (unsigned long) size);
   for (size_t i = 0; i < size; i++)
   {
      rox_log("%.16f ", input[i]);
   }
   rox_log("\n");

   return error;
}

int rox_ansi_array_float_print ( float * input, size_t size )
{
   int error = 0;

   rox_log("Array (%lu) : \r\n", (unsigned long) size);
   for (size_t i = 0; i < size; i++)
   {
      rox_log("%.16f ", input[i]);
   }
   rox_log("\n");

   return error;
}

int rox_ansi_array_uint_print ( unsigned int * input, size_t size )
{
   int error = 0;

   rox_log("Array (%lu) : \r\n", (unsigned long) size);
   for ( Rox_Sint i = 0; i < size; i++)
   {
      rox_log("%d ", input[i]);
   }
   rox_log("\n");

   return error;
}

int rox_ansi_array_double_print_precision ( double * input, size_t size, int precision )
{
   int error = 0;
   char format[16];

   sprintf(format, "%%.%d%c ", precision, 'f');

   rox_log("Array (%lu) : \r\n", (unsigned long) size);
   for ( Rox_Sint i = 0; i < size; i++)
   {
      rox_log(format, input[i]);
   }
   rox_log("\n");

   return error;
}

int rox_ansi_array_float_print_as_array2d ( float * input, size_t rows, size_t cols )
{
   int error = 0;

   rox_log("Array (%lu) in matrix form (%lu x %lu): \r\n", (unsigned long) rows*cols, (unsigned long) rows, (unsigned long) cols);
   for (size_t v = 0; v < rows; v++)
   {
      for (size_t u = 0; u < cols; u++)
      {
         rox_log("%.8f ", input[v*cols+u]);
      }
      rox_log("\n");
   }
   rox_log("\n");

   return error;
}

int rox_ansi_array_double_print_as_array2d ( double * input, size_t rows, size_t cols )
{
   int error = 0;

   rox_log("Array (%lu) in matrix form (%lu x %lu): \r\n", (unsigned long) rows*cols, (unsigned long) rows, (unsigned long) cols);
   for (size_t v = 0; v < rows; v++)
   {
      for (size_t u = 0; u < cols; u++)
      {
         rox_log("%.16f ", input[v*cols+u]);
      }
      rox_log("\n");
   }
   rox_log("\n");

   return error;
}
