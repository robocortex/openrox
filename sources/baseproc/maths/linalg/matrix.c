//==============================================================================
//
//    OPENROX   : File matrix.c
//
//    Contents  : Implementation of matrix module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matrix.h"

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>

#include <baseproc/geometry/transforms/transform_tools.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matrix_new ( Rox_Matrix * matrix, const Rox_Sint rows, const Rox_Sint cols)
{
   return rox_array2d_double_new(matrix, rows, cols);
}

Rox_ErrorCode rox_matrix_del ( Rox_Matrix* matrix)
{
   return rox_array2d_double_del(matrix);
}

Rox_ErrorCode rox_matrix_get_cols ( Rox_Sint * cols, const Rox_Matrix matrix )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!cols) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_get_cols(cols, matrix);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_matrix_get_rows ( Rox_Sint * rows, const Rox_Matrix matrix)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rows)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_get_rows(rows, matrix);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_matrix_get_value ( Rox_Double * value, const Rox_Matrix matrix, const Rox_Sint i, const Rox_Sint j)
{
   return rox_array2d_double_get_value ( value, matrix, i, j);
}

Rox_ErrorCode rox_matrix_set_value ( Rox_Matrix matrix, const Rox_Sint i, const Rox_Sint j, const Rox_Double value)
{
   return rox_array2d_double_set_value(matrix, i, j, value);
}

Rox_ErrorCode rox_matrix_set_unit(Rox_Matrix matrix)
{
   return rox_array2d_double_fillunit(matrix);
}

Rox_ErrorCode rox_matrix_set_zero(Rox_Matrix matrix)
{
   return rox_array2d_double_fillval(matrix, 0.0);
}

Rox_ErrorCode rox_matrix_print(Rox_Matrix matrix)
{
   return rox_array2d_double_print_precision(matrix, 15);
}

Rox_ErrorCode rox_matrix_build_calibration_matrix (
   Rox_Matrix matrix, 
   const Rox_Double fu, 
   const Rox_Double fv, 
   const Rox_Double cu, 
   const Rox_Double cv
)
{
   return rox_transformtools_build_calibration_matrix ( matrix, fu, fv, cu, cv );
}

Rox_ErrorCode rox_matrix_copy ( Rox_Matrix matrix_out, const Rox_Matrix matrix_inp )
{
   return rox_array2d_double_copy(matrix_out, matrix_inp);
}

// matrix_out = matrix_inp1 + matrix_inp2'
Rox_ErrorCode rox_matrix_add_matmattrans (
   Rox_Matrix matrix_out, 
   const Rox_Matrix matrix_in1, 
   const Rox_Matrix matrix_in2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!matrix_out || !matrix_in1 || !matrix_in2) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, matrix_out); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols != rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   error = rox_array2d_double_check_size ( matrix_in1, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( matrix_in2, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** matrix_out_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&matrix_out_data, matrix_out);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** matrix_in1_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&matrix_in1_data, matrix_in1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** matrix_in2_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&matrix_in2_data, matrix_in2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         matrix_out_data[i][j] = matrix_in1_data[i][j] + matrix_in2_data[j][i];
      }
   }
   
function_terminate:
   return error;
}

// matrix_out = matrix_inp1 + matrix_inp2'
Rox_ErrorCode rox_matrix_power (
   Rox_Matrix matrix_power, 
   const Rox_Matrix matrix_base, 
   const Rox_Sint exponent
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Matrix matrix_temp = NULL;

   if (!matrix_power || !matrix_base) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, matrix_power); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols != rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   error = rox_array2d_double_check_size(matrix_power, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(matrix_base, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new(&matrix_temp, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_unit(matrix_temp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   for (Rox_Sint k = 0; k < exponent; k++)
   {
      error = rox_array2d_double_mulmatmat(matrix_power, matrix_temp, matrix_base);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matrix_copy(matrix_temp, matrix_power);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   
function_terminate:
   rox_matrix_del(&matrix_temp);
   return error;
}

Rox_ErrorCode rox_matrix_trace (
   Rox_Double * trace, 
   const Rox_Matrix matrix
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
      
   if (!matrix || !trace) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, matrix); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Check if matrix is square
   if (cols != rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** matrix_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&matrix_data, matrix);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *trace = 0.0;
   for (Rox_Sint i = 0; i < rows; i++)
   {
      *trace += matrix_data[i][i];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_matrix_get_data_pointer_to_pointer ( 
   Rox_Double *** rowsptr, 
   const Rox_Matrix matrix 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matrix || !rowsptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer ( rowsptr, matrix );

function_terminate:
   return error;
}

Rox_ErrorCode rox_matrix_set_data( Rox_Matrix matrix, const Rox_Double * data )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !matrix || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_set_buffer_no_stride ( matrix, data );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_matrix_get_data ( Rox_Double * data, const Rox_Matrix matrix )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !matrix || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Sint cols =0, rows =0;
   error = rox_matrix_get_rows ( &rows, matrix );
   ROX_ERROR_CHECK_TERMINATE( error );
   
   error = rox_matrix_get_cols ( &cols, matrix );
   ROX_ERROR_CHECK_TERMINATE( error );
   
   Rox_Double ** matrix_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&matrix_data, matrix);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint r=0; r<rows; r++)
   {
      for (Rox_Sint c=0; c<cols; c++)
      {
         data[r*cols+c] = matrix_data[r][c];
      }
   }

function_terminate:
   return error;
}
