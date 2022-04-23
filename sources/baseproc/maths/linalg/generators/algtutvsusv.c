//============================================================================
//
//    OPENROX   : File tutvsusvgenerator.h
//
//    Contents  : API of tutvsusvgenerator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "algtutvsusv.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/multiply/mulmatmat.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linalg_tutvsusvupdate_right(Rox_Array2D_Double matrix, Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double buffer = NULL, update = NULL;
   
   Rox_Sint rows = 0; 
   error = rox_array2d_double_get_rows(&rows, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows < 4) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&buffer, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&update, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dv, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** du = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &du, update);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double a = dv[0][0];
   Rox_Double b = dv[1][0];
   Rox_Double c = dv[2][0];
   Rox_Double d = dv[3][0];

   if (fabs(c) < DBL_EPSILON && fabs(d) < DBL_EPSILON)
   {
      du[0][0] = 1;
      du[0][1] = 0;
      du[0][2] = a;
      du[1][0] = 0;
      du[1][1] = 1;
      du[1][2] = b;
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = 1;
   }
   else if (fabs(c-d) < DBL_EPSILON)
   {
      du[0][0] = exp(d);
      du[0][1] = 0;
      du[0][2] = a * exp(d);
      du[1][0] = 0;
      du[1][1] = exp(-2.0 * d);
      du[1][2] = (1.0 / 3.0) * (b * (-exp(-2.0* d) + exp(d)) / d);
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = exp(d);
   }
   else if (fabs(c + 0.2e1*d) < DBL_EPSILON)
   {
      du[0][0] = exp(c);
      du[0][1] = 0;
      du[0][2] = (1.0 / 3.0) * (a * (-exp(-2.0* d) + exp(d)) / d);
      du[1][0] = 0;
      du[1][1] = exp(d);
      du[1][2] = b * exp(d);
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = exp(d);
   }
   else
   {
      du[0][0] = exp(c);
      du[0][1] = 0;
      du[0][2] = -a * (exp(d) - exp(c)) / (c - d);
      du[1][0] = 0;
      du[1][1] = exp(-c - d);
      du[1][2] = b * (-exp(-c - d) + exp(d)) / (c + 0.2e1 * d);
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = exp(d);
   }

   error = rox_array2d_double_mulmatmat(buffer, matrix, update); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(matrix, buffer); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&buffer);
   rox_array2d_double_del(&update);

   return error;
}

Rox_ErrorCode rox_linalg_tutvsusvupdate_left(Rox_Array2D_Double matrix, Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double buffer = NULL, update = NULL;

   Rox_Sint rows = 0; 
   error = rox_array2d_double_get_rows(&rows, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows < 4) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&buffer, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&update, 3,3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dv, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** du = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &du, update);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double a = dv[0][0];
   Rox_Double b = dv[1][0];
   Rox_Double c = dv[2][0];
   Rox_Double d = dv[3][0];

   if (fabs(c) < DBL_EPSILON && fabs(d) < DBL_EPSILON)
   {
      du[0][0] = 1;
      du[0][1] = 0;
      du[0][2] = a;
      du[1][0] = 0;
      du[1][1] = 1;
      du[1][2] = b;
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = 1;
   }
   if (fabs(c-d) < DBL_EPSILON)
   {
      du[0][0] = exp(d);
      du[0][1] = 0;
      du[0][2] = a * exp(d);
      du[1][0] = 0;
      du[1][1] = exp(-2.0 * d);
      du[1][2] = (1.0 / 3.0) * (b * (-exp(-2.0* d) + exp(d)) / d);
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = exp(d);
   }
   else if (fabs(c + 0.2e1*d) < DBL_EPSILON)
   {
      du[0][0] = exp(c);
      du[0][1] = 0;
      du[0][2] = (1.0 / 3.0) * (a * (-exp(-2.0* d) + exp(d)) / d);
      du[1][0] = 0;
      du[1][1] = exp(d);
      du[1][2] = b * exp(d);
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = exp(d);
   }
   else
   {
      du[0][0] = exp(c);
      du[0][1] = 0;
      du[0][2] = -a * (exp(d) - exp(c)) / (c - d);
      du[1][0] = 0;
      du[1][1] = exp(-c - d);
      du[1][2] = b * (-exp(-c - d) + exp(d)) / (c + 0.2e1 * d);
      du[2][0] = 0;
      du[2][1] = 0;
      du[2][2] = exp(d);
   }

   error = rox_array2d_double_mulmatmat(buffer, update, matrix); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(matrix, buffer); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&buffer);
   rox_array2d_double_del(&update);

   return error;
}
