//==============================================================================
//
//    OPENROX   : File ansi_scale.c
//
//    Contents  : Implementation of ansi scale module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_scale.h"

int rox_ansi_array_float_scale_inplace ( float * inpout_data, const int size, const float scale )
{
   int error = 0;
   
   for ( int k = 0; k < size; k++)
   {
      inpout_data[k] *= scale;
   }
   
   return error;
}
