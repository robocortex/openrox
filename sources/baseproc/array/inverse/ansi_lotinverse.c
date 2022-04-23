//==============================================================================
//
//    OPENROX   : File ansi_lotinverse.c
//
//    Contents  : Implementation of ansi lotinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_lotinverse.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

int rox_ansi_array_float_lotinverse ( float * Li_data, const float * L_data, int n )
{
   int error = 0;

   // Set output matrix Ki to zero
   for ( int k = 0; k < n*n; k++ )
   {
      Li_data[k] = 0.0;
   }

   // inverse L
   for ( int i = 0; i < n; i++)
   {
      // We should test L_data[i*n+i] close to zero
      if ( fabs(L_data[i*n+i]) < FLT_MIN )
      { error = -1; goto function_terminate;}

      Li_data[i*n+i] = 1/L_data[i*n+i];
      for ( int j = 0; j < i; j++)
      {
         float s = 0.0;
         for ( int k = j; k <= i; k++)
         {
            s += L_data[i*n+k] * Li_data[k*n+j];
         }
         Li_data[i*n+j] = -s*Li_data[i*n+i];
      }
   }

function_terminate:
   return error;
}
