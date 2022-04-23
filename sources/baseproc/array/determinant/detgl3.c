//==============================================================================
//
//    OPENROX   : File detgl3.c
//
//    Contents  : Implementation of detgl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "detgl3.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_detgl3(Rox_Double *ptrDet, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptrDet || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double determinant = (
                    (data[0][0] * data[1][1] * data[2][2]) +
                    (data[0][1] * data[1][2] * data[2][0]) +
                    (data[0][2] * data[1][0] * data[2][1]) -
                    (data[0][0] * data[1][2] * data[2][1]) -
                    (data[0][1] * data[1][0] * data[2][2]) -
                    (data[0][2] * data[1][1] * data[2][0])
                 );

   *ptrDet = determinant;

function_terminate:
   return error;
}
