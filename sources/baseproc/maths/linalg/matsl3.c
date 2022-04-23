//==============================================================================
//
//    OPENROX   : File matsl3.c
//
//    Contents  : Implementation of matsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#include "matsl3.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/maths/linalg/generators/algsl3.h>
#include <baseproc/array/scale/scale.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matsl3_new ( Rox_MatSL3 * matsl3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSL3 ret = NULL;

   if (!matsl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *matsl3 = NULL;

   error = rox_array2d_double_new(&ret, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(ret);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *matsl3 = ret;
   
function_terminate:
   if(error) rox_array2d_double_del(&ret);
   return error;
}

Rox_ErrorCode rox_matsl3_del ( Rox_MatSL3 * matsl3 )
{
   return rox_array2d_double_del(matsl3);
}

Rox_ErrorCode rox_matsl3_copy ( Rox_MatSL3 result, const Rox_MatSL3 input)
{
    return rox_array2d_double_copy(result, input);
}

Rox_ErrorCode rox_matsl3_set_unit ( Rox_MatSL3 matsl3)
{
   return rox_array2d_double_fillunit(matsl3);
}

Rox_ErrorCode rox_matsl3_mulmatmat ( Rox_MatSL3 result, const Rox_MatSL3 input_1, const Rox_MatSL3 input_2)
{
   return rox_array2d_double_mulmatmat(result, input_1, input_2);
}

#if 0
Rox_ErrorCode rox_matsl3_inv(Rox_MatSL3 result, const Rox_MatSL3 input)
{
   return rox_array2d_double_svdinverse(result, input);
}
#else
Rox_ErrorCode rox_matsl3_inv ( Rox_MatSL3 result, const Rox_MatSL3 input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!result || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** H = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Hi = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Hi, result);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO : No need to divide by the determinant since it must be 1 for an SL3 matrix
   Rox_Double det = H[0][0]*H[1][1]*H[2][2] - H[0][0]*H[1][2]*H[2][1] - H[0][1]*H[1][0]*H[2][2] + H[0][1]*H[1][2]*H[2][0] + H[0][2]*H[1][0]*H[2][1] - H[0][2]*H[1][1]*H[2][0];

   Hi[0][0] = H[1][1]*H[2][2] - H[1][2]*H[2][1]; Hi[0][1] = H[0][2]*H[2][1] - H[0][1]*H[2][2]; Hi[0][2] = H[0][1]*H[1][2] - H[0][2]*H[1][1];
   Hi[1][0] = H[1][2]*H[2][0] - H[1][0]*H[2][2]; Hi[1][1] = H[0][0]*H[2][2] - H[0][2]*H[2][0]; Hi[1][2] = H[0][2]*H[1][0] - H[0][0]*H[1][2];
   Hi[2][0] = H[1][0]*H[2][1] - H[1][1]*H[2][0]; Hi[2][1] = H[0][1]*H[2][0] - H[0][0]*H[2][1]; Hi[2][2] = H[0][0]*H[1][1] - H[0][1]*H[1][0];

   error = rox_array2d_double_scale_inplace(result, 1.0/det);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
#endif

Rox_ErrorCode rox_matsl3_print ( const Rox_MatSL3 matsl3 )
{
   return rox_array2d_double_print_precision(matsl3, 16);
}

Rox_ErrorCode rox_matsl3_set_data ( Rox_MatSL3 matsl3, const Rox_Double data[9] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matsl3 || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matsl3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dp[0][0] = data[0];  dp[0][1] = data[1];  dp[0][2] = data[2];
   dp[1][0] = data[3];  dp[1][1] = data[4];  dp[1][2] = data[5];
   dp[2][0] = data[6];  dp[2][1] = data[7];  dp[2][2] = data[8];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matsl3_get_data(Rox_Double data[9], const Rox_MatSL3 matsl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matsl3 || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, matsl3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   data[0] = dp[0][0];  data[1] = dp[0][1];  data[2] = dp[0][2];
   data[3] = dp[1][0];  data[4] = dp[1][1];  data[5] = dp[1][2];
   data[6] = dp[2][0];  data[7] = dp[2][1];  data[8] = dp[2][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matsl3_get_data_pointer_to_pointer(Rox_Double *** rowsptr, const Rox_MatSL3 matsl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matsl3 || !rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(rowsptr, matsl3);

function_terminate:
   return error;
}

Rox_ErrorCode rox_matsl3_update_right(Rox_MatSL3 matsl3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new(&algebra, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&update, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_sl3generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_expmat_sl3(update, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(algebra, matsl3, update); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(matsl3, algebra); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   ROX_ERROR_CHECK(rox_array2d_double_del(&algebra));
   ROX_ERROR_CHECK(rox_array2d_double_del(&update));

   return error;
}

Rox_ErrorCode rox_matsl3_update_left(Rox_MatSL3 matsl3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL, update = NULL;

   error = rox_array2d_double_new(&algebra, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&update, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_linalg_sl3generator(algebra, vector); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_expmat_sl3(update, algebra); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(algebra, update, matsl3); 
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_copy(matsl3, algebra); 
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   ROX_ERROR_CHECK(rox_array2d_double_del(&algebra));
   ROX_ERROR_CHECK(rox_array2d_double_del(&update));

   return error;
}

Rox_ErrorCode rox_matsl3_mulmatinv( Rox_MatSL3 result, Rox_MatSL3 input_1, Rox_MatSL3 input_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSL3 inv_input_2 = NULL;

   if ( !result || !input_1 || !input_2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matsl3_new( &inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matsl3_inv( inv_input_2, input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat( result, input_1, inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_matsl3_del( &inv_input_2 ) );
   return error;
}

Rox_ErrorCode rox_array2d_double_expmat_sl3(Rox_Array2D_Double dest, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double f0 = 0.0, f1 = 0.0, f2 = 0.0;
   Rox_Double dt = 0.0, tt = 0.0, dn = 0.0;
   Rox_Double lr = 0.0, li = 0.0, dd = 0.0;
   Rox_Double a11, a12, a13;
   Rox_Double a21, a22, a23;
   Rox_Double a31, a32, a33;

   if (dest == 0 || input == 0)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(dest, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dout, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dinp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dinp, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   a11 = dinp[0][0]; a12 = dinp[0][1]; a13 = dinp[0][2];
   a21 = dinp[1][0]; a22 = dinp[1][1]; a23 = dinp[1][2];
   a31 = dinp[2][0]; a32 = dinp[2][1]; a33 = dinp[2][2];

   // In SL(3) A^3 = (trace(A^2)-trace(A)^2)/2 * A + det(A)
   // since trace(A) = 0
   // Solve cubic equation:
   // see: http://mathworld.wolfram.com/CubicFormula.html
   // scale factors (qn = 2*q and pn = 3*p) are used to eliminate division by 27 and 4
   // x^3 + 3 tt x + 2 dt = 0
   // where dt = - det(A)/2
   // and   tt = - ((1/2) (trace(A^2) - trace(A)^2))/3

   dt = (a21 * (a12 * a33 - a32 * a13) - a11 * (a22 * a33 - a32 * a23) - a31 * (a12 * a23 - a22 * a13)) / 2;
   tt = (a11 * a33 - a12 * a21 - a13 * a31 + a22 * a33 + a11 * a22 - a23 * a32) / 3;

   // polynomial discriminant dd = dt^2+tt^3 
   dd = (tt * tt * tt) + (dt * dt);

   if (dd < -1e-16)
   {
      // All roots are real and different
      Rox_Double de, phi;
      Rox_Double l1,   l2,   l3 ;
      Rox_Double l1_2, l2_2, l3_2;

      // Safe division : it is sure that tt is negative (not zero)
      phi = acos(dt / sqrt(-tt * tt * tt));

      // Eigenvalues 
      l1 = -2.0 * sqrt(-tt) * cos((phi + 0.0 * ROX_PI) / 3.0);    // l1 = -2.0*sqrt(-tt)*(cos(phi/3.0));                  
      l2 = -2.0 * sqrt(-tt) * cos((phi + 2.0 * ROX_PI) / 3.0);    // l2 =      sqrt(-tt)*(cos(phi/3)+sqrt(3)*sin(phi/3)); 
      l3 = -2.0 * sqrt(-tt) * cos((phi + 4.0 * ROX_PI) / 3.0);    // l3 =      sqrt(-tt)*(cos(phi/3)-sqrt(3)*sin(phi/3)); 

      // Squared eigenvalues
      l1_2 = l1 * l1;
      l2_2 = l2 * l2;
      l3_2 = l3 * l3;

      // Safe division : it is sure that de is not zero
      de = (l1 - l3) * (l2 - l3) * (l1 - l2);

      // Coefficients 
      f0 = + (l1_2 * (l2 * exp(l3) - l3 * exp(l2)) + l2_2 * (l3 * exp(l1) - l1 * exp(l3)) + l3_2 * (l1 * exp(l2) - l2 * exp(l1))) / de;
      f1 = - (l1_2 * (exp(l3) -   exp(l2)) + l2_2 * (exp(l1) -   exp(l3)) + l3_2 * (exp(l2) -   exp(l1))) / de;
      f2 = + (l1  * (exp(l3) -   exp(l2)) + l2  * (exp(l1) -   exp(l3)) + l3  * (exp(l2) -   exp(l1))) / de;
   }
   else
   {
      Rox_Double sinc_li = 0.0;
      Rox_Double sinc_li_2 = 0.0;
      Rox_Double x = 0.0;

      // One real root and two complex conjugate 
      x = sqrt(fabs(dd)) - dt;
      if (x > 0.0) dn = pow(x, 1.0 / 3.0);
      else if (x < 0.0) dn = -pow(-x, 1.0 / 3.0);
      else dn = 0.0;

      if (fabs(dn) > DBL_MIN)
      {
         lr = - (dn - tt / dn) / 2.0;
         li = - (dn + tt / dn) / 2.0 * sqrt(3.0);

         // Compute cardinal sinus of li and li / 2 
         if (li < DBL_EPSILON)
         {
            sinc_li = 1.0;
         }
         else
         {
            sinc_li = sin(li) / li;
         }

         if (0.5 * li < DBL_EPSILON)
         {
            sinc_li_2 = 1.0;
         }
         else
         {
            sinc_li_2 = sin(li * 0.5) / (li * 0.5);
         }
      }
      else
      {
         lr = 0.0;
         li = 0.0;
         sinc_li = 1.0;
         sinc_li_2 = 1.0;
      }

      if (fabs(lr) > 1e-16)
      {
         Rox_Double li2 = li * li;
         Rox_Double lr2 = lr * lr;
         Rox_Double lr3 = lr * lr2;

         // Coefficients 
         f0 = exp(lr) * (li2 * exp(-3 * lr) + lr2 * exp(-3 * lr) - 6 * lr3 * sinc_li + 8 * lr2 * cos(li) + 2 * li * lr * sin(li)) / (9 * lr2 + li2);
         f1 = exp(lr) * (li * sin(li) + 3 * lr2 * sinc_li + 2 * lr * cos(li) - 2 * lr * exp(-3 * lr)) / (9 * lr2 + li2);
         f2 = exp(lr) * (exp(-3 * lr) - cos(li) + 3 * lr * sinc_li) / (9 * lr2 + li2);
      }
      else
      {
         f0 = 1.0;
         f1 = sinc_li; // sin(li)/li 
         f2 = 0.5 * sinc_li_2 * sinc_li_2; // (sin(li/2)/(li/2))^2/2 
      }
   }
   // Build result matrix 

   // compute H = f0*I + f1*A + f2*A^2 
   dout[0][0] = f0 + f1 * a11 + f2 * (a11 * a11 + a12 * a21 + a13 * a31);
   dout[0][1] =      f1 * a12 + f2 * (a11 * a12 + a12 * a22 + a13 * a32);
   dout[0][2] =      f1 * a13 + f2 * (a11 * a13 + a12 * a23 + a13 * a33);
   dout[1][0] =      f1 * a21 + f2 * (a21 * a11 + a22 * a21 + a23 * a31);
   dout[1][1] = f0 + f1 * a22 + f2 * (a21 * a12 + a22 * a22 + a23 * a32);
   dout[1][2] =      f1 * a23 + f2 * (a21 * a13 + a22 * a23 + a23 * a33);
   dout[2][0] =      f1 * a31 + f2 * (a31 * a11 + a32 * a21 + a33 * a31);
   dout[2][1] =      f1 * a32 + f2 * (a31 * a12 + a32 * a22 + a33 * a32);
   dout[2][2] = f0 + f1 * a33 + f2 * (a31 * a13 + a32 * a23 + a33 * a33);

function_terminate:
   return error;
}


Rox_ErrorCode rox_matsl3_check_size ( const Rox_MatSL3 matsl3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_check_size ( matsl3, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}