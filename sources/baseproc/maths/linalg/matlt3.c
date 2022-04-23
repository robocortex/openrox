//==============================================================================
//
//    OPENROX   : File matlt3.c
//
//    Contents  : Implementation of matlt3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#include "matlt3.h"

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/maths/linalg/generators/alglt3.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matlt3_new(Rox_MatLT3 * matlt3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatLT3 ret = NULL;

   if (!matlt3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *matlt3 = NULL;

   error = rox_array2d_double_new(&ret, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(ret);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *matlt3 = ret;
   
function_terminate:
   if(error) rox_array2d_double_del(&ret);
   return error;
}

Rox_ErrorCode rox_matlt3_del(Rox_MatLT3 *matlt3)
{
   return rox_array2d_double_del(matlt3);
}

Rox_ErrorCode rox_matlt3_copy(Rox_MatLT3 result, const Rox_MatLT3 input)
{
    return rox_array2d_double_copy(result, input);
}

Rox_ErrorCode rox_matlt3_set_unit(Rox_MatLT3 matlt3)
{
   return rox_array2d_double_fillunit(matlt3);
}

Rox_ErrorCode rox_matlt3_mulmatmat(Rox_MatLT3 result, const Rox_MatLT3 input_1, const Rox_MatLT3 input_2)
{
   return rox_array2d_double_mulmatmat(result, input_1, input_2);
}

Rox_ErrorCode rox_matlt3_inv(Rox_MatLT3 result, const Rox_MatLT3 input)
{
   return rox_array2d_double_svdinverse(result, input);
}

Rox_ErrorCode rox_matlt3_print(Rox_MatLT3 matlt3)
{
   return rox_array2d_double_print_precision(matlt3, 16);
}

Rox_ErrorCode rox_matlt3_set_data(Rox_MatLT3 matlt3, const Rox_Double data[9])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matlt3 || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matlt3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dp[0][0] = data[0];  dp[0][1] =     0.0;  dp[0][2] =     0.0;
   dp[1][0] = data[3];  dp[1][1] = data[4];  dp[1][2] =     0.0;
   dp[2][0] = data[6];  dp[2][1] = data[7];  dp[2][2] = data[8];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matlt3_get_data(Rox_Double data[9], const Rox_MatLT3 matlt3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matlt3 || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matlt3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   data[0] = dp[0][0];  data[1] =      0.0;  data[2] =      0.0;
   data[3] = dp[1][0];  data[4] = dp[1][1];  data[5] =      0.0;
   data[6] = dp[2][0];  data[7] = dp[2][1];  data[8] = dp[2][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matlt3_get_data_pointer_to_pointer ( Rox_Double *** rowsptr, const Rox_MatLT3 matlt3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matlt3 || !rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(rowsptr, matlt3);

function_terminate:
   return error;
}

Rox_ErrorCode rox_matlt3_update_right ( Rox_MatLT3 matlt3, const Rox_Array2D_Double vector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new(&algebra, 3,3); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&update, 3,3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_linalg_lt3generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   //error = rox_array2d_double_expmat_ut3(update, algebra); 
   //ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_mulmatmat(algebra, matlt3, update); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_copy(matlt3, algebra); 
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   ROX_ERROR_CHECK(rox_array2d_double_del(&algebra));
   ROX_ERROR_CHECK(rox_array2d_double_del(&update));

   return error;
}

Rox_ErrorCode rox_matlt3_update_left(Rox_MatLT3 matlt3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new(&algebra, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&update, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_linalg_lt3generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_scale_inplace(algebra, -1.0);
   ROX_ERROR_CHECK_TERMINATE( error );

   //error = rox_array2d_double_expmat_lt3(update, algebra); 
   //ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(algebra, update, matlt3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_copy(matlt3, algebra); 
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   ROX_ERROR_CHECK(rox_array2d_double_del(&algebra));
   ROX_ERROR_CHECK(rox_array2d_double_del(&update));

   return error;
}

Rox_ErrorCode rox_matlt3_mulmatinv( Rox_MatLT3 result, const Rox_MatLT3 input_1, const Rox_MatLT3 input_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatLT3 inv_input_2 = NULL;

   if ( !result || !input_1 || !input_2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matlt3_new( &inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matlt3_inv( inv_input_2, input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat( result, input_1, inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_matlt3_del( &inv_input_2 ) );
   return error;
}
