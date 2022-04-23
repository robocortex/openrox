//==============================================================================
//
//    OPENROX   : File ansi_array2d_print.c
//
//    Contents  : Implementation of ansi array2d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_array2d_print.h"

#include <stdio.h>

int rox_ansi_array2d_float_print ( float ** input_data, int rows, int cols )
{
   for ( int i = 0; i < rows; i++)
   {
      for ( int j = 0; j < cols; j++)
      {
         printf("%.16f, ", input_data[i][j]);
      }
      printf("\n");
   }
   return 0;
}
