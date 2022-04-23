//==============================================================================
//
//    OPENROX   : File normalize.c
//
//    Contents  : Implementation of vector normalization module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "normalize.h"

#include <math.h>

#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/scale/scale.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_normalize(Rox_Array2D_Double res, Rox_Array2D_Double inp) 
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double norm2 = 0.0;
   
   error = rox_array2d_double_norm2sq(&norm2, inp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(res, inp, 1.0/sqrt(norm2));
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   
   return error;
}