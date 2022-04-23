//==============================================================================
//
//    OPENROX   : File expmat.c
//
//    Contents  : Implementation of expmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "expmat.h"

#include <baseproc/array/add/add.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_expmat(Rox_Array2D_Double dest, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint s = 0, e = 0, q = 6 ;
   Rox_Double norm = 0.0, c = 0.0;
   Rox_Bool p = 1;
   Rox_Double sum = 0.0;
   Rox_Array2D_Double inputScaled = NULL, T = NULL, X = NULL, Xtemp = NULL, D = NULL, Di = NULL, cX = NULL;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check if input is a square matrix
   if (cols != rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check if output has the same size of input
   error = rox_array2d_double_check_size(dest, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&inputScaled, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&T, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&X, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Xtemp, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&D, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Di, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&cX, cols, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute input norm
   Rox_Double ** din = NULL;

   error = rox_array2d_double_get_data_pointer_to_pointer ( &din, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   norm = 0.0;
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      sum = 0.0;
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         sum += fabs(din[i][j]);
      }
      if (sum > norm) norm = sum;
   }

   frexp(norm, &e);
   s = ROX_MAX(0, e + 1);
   c = 1.0 / pow(2.0, s);

   error = rox_array2d_double_scale(inputScaled, input, c);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // dest = I
   error = rox_array2d_double_fillunit(dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // D = I
   error = rox_array2d_double_fillunit(D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   c = 0.5;
   error = rox_array2d_double_copy(X, inputScaled);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // cX = c * X
   error = rox_array2d_double_scale(cX, X, c);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // dest = dest + cX
   error = rox_array2d_double_add(dest, dest, cX);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // D = D - cX
   error = rox_array2d_double_substract(D, D, cX);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k = 2; k <= q; k++)
   {
      c = c * ((Rox_Double) q - k + 1) / ((Rox_Double) k * (2 * q - k + 1));

      error = rox_array2d_double_mulmatmat(Xtemp, inputScaled, X); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_copy(X, Xtemp); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_scale(cX, X, c); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(dest, dest, cX);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (p)
      {
         error = rox_array2d_double_add(D, D, cX);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         error = rox_array2d_double_substract(D, D, cX);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      p = !p;
   }

   error = rox_array2d_double_svdinverse(Di, D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(Xtemp, Di, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(dest, Xtemp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k = 1; k <= s; k++)
   {
      error = rox_array2d_double_mulmatmat(T, dest, dest);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_copy(dest, T);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   ROX_ERROR_CHECK(rox_array2d_double_del(&inputScaled))
   ROX_ERROR_CHECK(rox_array2d_double_del(&T))
   ROX_ERROR_CHECK(rox_array2d_double_del(&X))
   ROX_ERROR_CHECK(rox_array2d_double_del(&Xtemp))
   ROX_ERROR_CHECK(rox_array2d_double_del(&D))
   ROX_ERROR_CHECK(rox_array2d_double_del(&Di))
   ROX_ERROR_CHECK(rox_array2d_double_del(&cX))

   return error;
}
