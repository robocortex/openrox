//============================================================================
//
//    OPENROX   : File algse2.h
//
//    Contents  : API of se2generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "algse2.h"

#include <baseproc/array/exponent/expmat.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linalg_se2generator(Rox_Array2D_Double algebra, Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!algebra) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!vector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(algebra, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(vector, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, algebra);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** din = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &din, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dout[0][0] = 0;
   dout[0][1] = -din[2][0];
   dout[0][2] = din[0][0];

   dout[1][0] = din[2][0];
   dout[1][1] = 0;
   dout[1][2] = din[1][0];

   dout[2][0] = 0;
   dout[2][1] = 0;
   dout[2][2] = 0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_linalg_se2update_right(Rox_Array2D_Double pose_se2, Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new(&algebra, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&update, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_se2generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_expmat(update, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(algebra, pose_se2, update); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(pose_se2, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&algebra);
   rox_array2d_double_del(&update);

   return error;
}

Rox_ErrorCode rox_linalg_se2update_left(Rox_Array2D_Double pose_se2, Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new(&algebra, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&update, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_se2generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_expmat(update, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(algebra, update, pose_se2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(pose_se2, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&algebra);
   rox_array2d_double_del(&update);

   return error;
}
