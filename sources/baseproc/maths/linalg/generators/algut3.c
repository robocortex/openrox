//============================================================================
//
//    OPENROX   : File algut3.h
//
//    Contents  : API of algut3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "algut3.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linalg_ut3generator ( Rox_Array2D_Double algebra, Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!algebra) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!vector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(algebra, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(vector, 6, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, algebra);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dinp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dinp, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dout[0][0] = dinp[0][0];
   dout[0][1] = dinp[1][0];
   dout[0][2] = dinp[2][0];

   dout[1][0] = 0.0;
   dout[1][1] = dinp[3][0];
   dout[1][2] = dinp[4][0];

   dout[2][0] = 0.0;
   dout[2][1] = 0.0;
   dout[2][2] = dinp[5][0];

function_terminate:
   return error;
}
