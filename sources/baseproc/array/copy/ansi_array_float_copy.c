//==============================================================================
//
//    OPENROX   : File ansi_array_float_copy.c
//
//    Contents  : Implementation of array2d_float_copy module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_array_float_copy.h"
#include <string.h>

int rox_ansi_array_float_copy ( float * out, const float * inp, const int size )
{
   int error = 0;

   memcpy ( out, inp, size * sizeof ( float ) );

   return error;
}