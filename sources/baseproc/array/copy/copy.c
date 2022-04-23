//==============================================================================
//
//    OPENROX   : File copy.c
//
//    Contents  : Implementation of copy module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "copy.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_vector_row_copy(Rox_Array2D_Double vR, Rox_Array2D_Double R)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   if (!vR || !R ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Rd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Rd, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** vRd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &vRd, vR );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&rows, &cols, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(vR, 1, rows*cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint j = 0; j < rows; j++)
   {
      for (Rox_Sint i = 0; i < cols; i++)
      {
         vRd[0][j*cols+i] = Rd[j][i];
      }
   }

function_terminate:

   return error;
}

Rox_ErrorCode rox_vector_col_copy(Rox_Array2D_Double vR, Rox_Array2D_Double R)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   if (!vR || !R ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Rd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Rd, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** vRd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &vRd, vR);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&rows, &cols, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(vR, rows*cols, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint j = 0; j < rows; j++)
   {
      for (Rox_Sint i = 0; i < cols; i++)
      {
         vRd[j*cols+i][0] = Rd[j][i];
      }
   }

function_terminate:

   return error;
}