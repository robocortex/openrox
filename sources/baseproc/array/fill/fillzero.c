//==============================================================================
//
//    OPENROX   : File fillzero.c
//
//    Contents  : Implementation of fillzero module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fillzero.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_fillzero ( Rox_Array2D_Double dest )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = 0.0;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_fillzero(Rox_Array2D_Float dest)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = 0.0f;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_fillzero ( Rox_Array2D_Uint dest )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !dest ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = 0;
      }
   }

function_terminate:
   return error;
}
