//==============================================================================
//
//    OPENROX   : File mat2x2inv.c
//
//    Contents  : Implementation of mat2x2inv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mat2x2inv.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/determinant/detgl2.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_mat2x2_inverse(Rox_Array2D_Double output, Rox_Array2D_Double matrix)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double det = 0.0, idet = 0.0;

   if (!output || !matrix) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size(matrix, 2, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_check_size(output, 2, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dm = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dm, matrix );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dout, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Determinant MUST not be 0
   error = rox_array2d_double_detgl2(&det, matrix);
   if (error || fabs(det) < DBL_EPSILON) 
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   
   idet = 1.0 / det;

   // Classic cofactors inverse
   dout[0][0] = idet * dm[1][1] ;
   dout[0][1] = idet * (-dm[0][1]);
   dout[1][0] = idet * (-dm[1][0]);
   dout[1][1] = idet * dm[0][0];
  
function_terminate:
   return error;
}
