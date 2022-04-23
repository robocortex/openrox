//==============================================================================
//
//    OPENROX   : File array2d_print.c
//
//    Contents  : Implementation of array2d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "complex_print.h"

#include <stdio.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_complex_print ( Rox_Complex complex )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   rox_log("%f + i * %f\n", complex->real, complex->imag);
   
   return error;
}
