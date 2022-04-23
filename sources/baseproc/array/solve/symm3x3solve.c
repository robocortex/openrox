//==============================================================================
//
//    OPENROX   : File symm3x3solve.c
//
//    Contents  : Implementation of symm3x3solve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "symm3x3solve.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_symm3x3_solve ( Rox_Array2D_Double output, Rox_Array2D_Double matrix, Rox_Array2D_Double vec )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !matrix || !vec) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(output, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(matrix, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(vec, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dm = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dm, matrix );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dout, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dvec = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dvec, vec );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double t1 = dm[1][2] * dm[1][2];
   Rox_Double t6 = dm[0][1] * dm[0][1];
   Rox_Double t7 = dm[0][2] * dm[0][2];
   Rox_Double t8 = -t7 * dm[1][1] + 2.0 * dm[0][2] * dm[0][1] * dm[1][2] + (-t6 + dm[0][0] * dm[1][1]) * dm[2][2] - dm[0][0] * t1;

   if (fabs(t8) < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double t2 = dvec[1][0] * dm[0][2];
   Rox_Double t3 = dvec[2][0] * dm[0][1];
   Rox_Double t4 = dvec[0][0] * dm[2][2];
   Rox_Double t5 = dvec[1][0] * dm[2][2];
   Rox_Double t9 = dvec[0][0] * dm[1][2];

   t8 = 1.0 / t8;

   dout[0][0] = ((t4 - dvec[2][0] * dm[0][2]) * dm[1][1] - dvec[0][0] * t1 + (t2 + t3) * dm[1][2] - t5 * dm[0][1]) * t8;
   dout[1][0] = - ((-t5 + dvec[2][0] * dm[1][2]) * dm[0][0] + dvec[1][0] * t7 + (-t9 - t3) * dm[0][2] + t4 * dm[0][1]) * t8;
   dout[2][0] = ((-dvec[1][0] * dm[1][2] + dvec[2][0] * dm[1][1]) * dm[0][0] - dvec[2][0] * t6 + (t9 + t2) * dm[0][1] - dvec[0][0] * dm[0][2] * dm[1][1]) * t8;

function_terminate:
   return error;
}
