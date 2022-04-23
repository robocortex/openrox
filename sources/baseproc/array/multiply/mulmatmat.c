//==============================================================================
//
//    OPENROX   : File mulmatmat.c
//
//    Contents  : Implementation of mulmatmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mulmatmat.h"
#include "ansi_mulmatmat.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_mulmatmat ( 
   Rox_Array2D_Double res, 
   Rox_Array2D_Double one, 
   Rox_Array2D_Double two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint res_cols = 0, res_rows = 0;
   error = rox_array2d_double_get_size ( &res_rows, &res_cols, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint one_cols = 0, one_rows = 0;
   error = rox_array2d_double_get_size ( &one_rows, &one_cols, one); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint two_cols = 0, two_rows = 0;
   error = rox_array2d_double_get_size ( &two_rows, &two_cols, two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (res_rows != one_rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (res_cols != two_cols) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (one_cols != two_rows) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** res_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &res_data, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** one_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &one_data, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** two_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &two_data, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!res_data || !one_data || !two_data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ansi_array2d_double_mulmatmat ( res_data, res_rows, res_cols, one_data, two_data, two_rows );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}