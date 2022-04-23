//==============================================================================
//
//    OPENROX   : File norm2sq.c
//
//    Contents  : Implementation of vector norm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "norm2sq.h"
#include <math.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_norm2 ( Rox_Double * norm2, Rox_Array2D_Double vector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double norm2sq = 0.0;

   error = rox_array2d_double_norm2sq(&norm2sq, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *norm2 = sqrt(norm2sq);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_norm2sq ( Rox_Double * norm, Rox_Array2D_Double vector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vector || !norm) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, vector);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   if (rows != 1 && cols != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dd1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dd1, vector );

   Rox_Double sum = 0.0;

   if (rows == 1)
   {
      for ( Rox_Sint i = 0; i < cols; i++)
      {
         sum += dd1[0][i] * dd1[0][i];
      }
   }
   else
   {
      for ( Rox_Sint i = 0; i < rows; i++)
      {
         sum += dd1[i][0] * dd1[i][0];
      }
   }

   *norm = sum;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_norm_frobenius(Rox_Double * norm, Rox_Array2D_Double matrix)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 0, rows = 0;
   Rox_Double sum = 0.0;

   if (!matrix || !norm) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_size(&rows, &cols, matrix);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data_rows = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data_rows, matrix );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Double *data_row = data_rows[i];
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         sum += data_row[j] * data_row[j];
      }
   }

   *norm = sqrt(sum);

function_terminate:
   return error;
}