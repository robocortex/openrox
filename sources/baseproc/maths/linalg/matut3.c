//==============================================================================
//
//    OPENROX   : File matut3.c
//
//    Contents  : Implementation of matut3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#include "matut3.h"

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/linalg/generators/algut3.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matut3_new ( Rox_MatUT3 * matut3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatUT3 ret = NULL;

   if (!matut3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *matut3 = NULL;

   error = rox_array2d_double_new ( &ret, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit ( ret );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *matut3 = ret;
   
function_terminate:
   if(error) rox_array2d_double_del ( &ret );
   return error;
}

Rox_ErrorCode rox_matut3_del ( Rox_MatUT3 * matut3 )
{
   return rox_array2d_double_del ( matut3 );
}

Rox_ErrorCode rox_matut3_copy ( Rox_MatUT3 result, const Rox_MatUT3 input )
{
   return rox_array2d_double_copy ( result, input );
}

Rox_ErrorCode rox_matut3_set_unit ( Rox_MatUT3 matut3 )
{
   return rox_array2d_double_fillunit ( matut3 );
}

Rox_ErrorCode rox_matut3_mulmatmat ( Rox_MatUT3 result, const Rox_MatUT3 input_1, const Rox_MatUT3 input_2)
{
   return rox_array2d_double_mulmatmat ( result, input_1, input_2);
}

#if 0
Rox_ErrorCode rox_matut3_inv(Rox_MatUT3 result, const Rox_MatUT3 input)
{
   return rox_array2d_double_svdinverse(result, input);
}
#else
Rox_ErrorCode rox_matut3_inv(Rox_MatUT3 result, const Rox_MatUT3 input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!result || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** U = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &U, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ui = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Ui, result);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double det = U[0][0]*U[1][1]*U[2][2];

   if (fabs(det) < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Ui[0][0] = U[1][1]*U[2][2]; Ui[0][1] = - U[0][1]*U[2][2]; Ui[0][2] = U[0][1]*U[1][2] - U[0][2]*U[1][1];
   Ui[1][0] =             0.0; Ui[1][1] =   U[0][0]*U[2][2]; Ui[1][2] =                 - U[0][0]*U[1][2];
   Ui[2][0] =             0.0; Ui[2][1] =               0.0; Ui[2][2] =                   U[0][0]*U[1][1];

   error = rox_array2d_double_scale_inplace ( result, 1.0/det );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
#endif

Rox_ErrorCode rox_matut3_print ( Rox_MatUT3 matut3 )
{
   return rox_array2d_double_print_precision(matut3, 16);
}

Rox_ErrorCode rox_matut3_set_data ( Rox_MatUT3 matut3, const Rox_Double data[9] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matut3 || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matut3);

   dp[0][0] = data[0];  dp[0][1] = data[1];  dp[0][2] = data[2];
   dp[1][0] =     0.0;  dp[1][1] = data[4];  dp[1][2] = data[5];
   dp[2][0] =     0.0;  dp[2][1] =     0.0;  dp[2][2] = data[8];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matut3_set_data_float ( Rox_MatUT3 matut3, const Rox_Float data[9] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !matut3 || !data ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matut3);

   dp[0][0] = (Rox_Double) data[0]; dp[0][1] = (Rox_Double) data[1]; dp[0][2] = (Rox_Double) data[2];
   dp[1][0] =                  0.0; dp[1][1] = (Rox_Double) data[4]; dp[1][2] = (Rox_Double) data[5];
   dp[2][0] =                  0.0; dp[2][1] =                  0.0; dp[2][2] = (Rox_Double) data[8];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matut3_get_data ( Rox_Double data[9], const Rox_MatUT3 matut3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matut3 || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matut3);

   data[0] = dp[0][0];  data[1] = dp[0][1];  data[2] = dp[0][2];
   data[3] =      0.0;  data[4] = dp[1][1];  data[5] = dp[1][2];
   data[6] =      0.0;  data[7] =      0.0;  data[8] = dp[2][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matut3_get_data_pointer_to_pointer (Rox_Double *** rowsptr, const Rox_MatUT3 matut3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matut3 || !rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(rowsptr, matut3);

function_terminate:
   return error;
}

Rox_ErrorCode rox_matut3_update_right(Rox_MatUT3 matut3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new ( &algebra, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new ( &update, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_ut3generator ( algebra, vector ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_expmat_ut3 ( update, algebra ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat ( algebra, matut3, update ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy ( matut3, algebra ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del ( &algebra );
   rox_array2d_double_del ( &update );

   return error;
}

Rox_ErrorCode rox_matut3_update_left ( Rox_MatUT3 matut3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new ( &algebra, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &update, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_ut3generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale_inplace(algebra, -1.0);
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_expmat_ut3(update, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(algebra, update, matut3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(matut3, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&algebra);
   rox_array2d_double_del(&update);

   return error;
}

Rox_ErrorCode rox_matut3_mulmatinv( Rox_MatUT3 result, const Rox_MatUT3 input_1, const Rox_MatUT3 input_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatUT3 inv_input_2 = NULL;

   if ( !result || !input_1 || !input_2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matut3_new( &inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_inv ( inv_input_2, input_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat ( result, input_1, inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matut3_del( &inv_input_2 );
   return error;
}

Rox_ErrorCode rox_array2d_double_expmat_ut3 ( Rox_MatUT3 dest, const Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest || !input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(dest, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dinp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dinp, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!dout || !dinp)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // A = [a b c; 0 d e; 0 0 f] 
   Rox_Double a = dinp[0][0];
   Rox_Double b = dinp[0][1];
   Rox_Double c = dinp[0][2];
   Rox_Double d = dinp[1][1];
   Rox_Double e = dinp[1][2];
   Rox_Double f = dinp[2][2];

   Rox_Double exp_a = exp(a);
   Rox_Double exp_d = exp(d);
   Rox_Double exp_f = exp(f);

   // Case 1: a != d 
   if (fabs(a - d) > DBL_EPSILON)
   {
      // Case 1: a == 0 
      if (fabs(a) < DBL_EPSILON)
      {
         dout[0][0] = 1.0;
         dout[0][1] = (b * (exp_d - 1.0)) / d;
         dout[0][2] = c - (b * e) / d + (b * e * (exp_d - 1)) / (d * d);

         dout[1][0] = 0.0;
         dout[1][1] = exp_d;
         dout[1][2] = (e * (exp_d - 1)) / d;

         dout[2][0] = 0.0;
         dout[2][1] = 0.0;
         dout[2][2] = exp_f;
      }

      // Case 2: d == 0 
      else if (fabs(d) < DBL_EPSILON)
      {
         dout[0][0] = exp_a;
         dout[0][1] = (b * (exp_a - 1.0)) / a;
         dout[0][2] = (b * e * (exp_a - 1)) / (a * a) - (c + b * e - c * exp_a) / a;

         dout[1][0] = 0.0;
         dout[1][1] = 1;
         dout[1][2] = e;

         dout[2][0] = 0.0;
         dout[2][1] = 0.0;
         dout[2][2] = exp_f;
      }

      // Case 3: a != 0 and d != 0
      else
      {
         dout[0][0] = exp_a;
         dout[0][1] = (b * (exp_a - exp_d)) / (a - d);
         dout[0][2] = (a * (b * e - b * e * exp_d) - d * (b * e - b * e * exp_a)) / (a * d * (a - d)) - (c - c * exp_a) / a;

         dout[1][0] = 0.0;
         dout[1][1] = exp_d;
         dout[1][2] = (e * (exp_d - 1)) / d;

         dout[2][0] = 0.0;
         dout[2][1] = 0.0;
         dout[2][2] = exp_f;
      }
   }

   // Case 2: a == d 
   else
   {
      // Case 1: a != 0 
      if (fabs(a) > DBL_EPSILON)
      {
         dout[0][0] = exp_a;
         dout[0][1] = b * exp_a;
         dout[0][2] = (c * exp_a - c + b * e * exp_a) / a - (b * e * (exp_a - 1)) / (a * a);

         dout[1][0] = 0.0;
         dout[1][1] = exp_a;
         dout[1][2] = (e * (exp_a - 1)) / a;

         dout[2][0] = 0.0;
         dout[2][1] = 0.0;
         dout[2][2] = exp_f;
      }

      // Case 2: a == 0
      else
      {
         dout[0][0] = 1.0;
         dout[0][1] = b;
         dout[0][2] = c + (b * e) / 2;

         dout[1][0] = 0.0;
         dout[1][1] = 1.0;
         dout[1][2] = e;

         dout[2][0] = 0.0;
         dout[2][1] = 0.0;
         dout[2][2] = exp_f;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_matut3_build_calibration_matrix (
   Rox_MatUT3 matut3, 
   Rox_Double fu, 
   Rox_Double fv, 
   Rox_Double cu, 
   Rox_Double cv
)
{
   return rox_transformtools_build_calibration_matrix ( matut3, fu, fv, cu, cv );
}

Rox_ErrorCode rox_matut3_check_size ( const Rox_MatUT3 matut3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_check_size ( matut3, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}