//==============================================================================
//
//    OPENROX   : File mat3x3inv.c
//
//    Contents  : Implementation of mat3x3inv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mat3x3inv.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/determinant/detgl3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_mat3x3_inverse ( Rox_Array2D_Double output, Rox_Array2D_Double matrix )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double det = 0.0, idet = 0.0;

   if (!output || !matrix) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size(matrix, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(output, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dm = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dm, matrix );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dout, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Determinant MUST not be 0
   error = rox_array2d_double_detgl3 ( &det, matrix );

   if (error || fabs(det) < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   idet = 1.0 / det;

   // Classic cofactors inverse
   dout[0][0] = idet * ( dm[1][1] * dm[2][2] - dm[1][2] * dm[2][1]);
   dout[0][1] = idet * (-dm[0][1] * dm[2][2] + dm[0][2] * dm[2][1]);
   dout[0][2] = idet * ( dm[0][1] * dm[1][2] - dm[0][2] * dm[1][1]);
   dout[1][0] = idet * ( dm[2][0] * dm[1][2] - dm[1][0] * dm[2][2]);
   dout[1][1] = idet * (-dm[2][0] * dm[0][2] + dm[0][0] * dm[2][2]);
   dout[1][2] = idet * ( dm[1][0] * dm[0][2] - dm[0][0] * dm[1][2]);
   dout[2][0] = idet * (-dm[2][0] * dm[1][1] + dm[1][0] * dm[2][1]);
   dout[2][1] = idet * ( dm[2][0] * dm[0][1] - dm[0][0] * dm[2][1]);
   dout[2][2] = idet * (-dm[1][0] * dm[0][1] + dm[0][0] * dm[1][1]);

function_terminate:
   return error;
}
