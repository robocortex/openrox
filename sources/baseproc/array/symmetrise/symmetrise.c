//==============================================================================
//
//    OPENROX   : File symmetrise.c
//
//    Contents  : Implementation of symmetrise module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "symmetrise.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_symmetrise_lower ( Rox_Array2D_Double array2d_lower )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !array2d_lower ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_double_get_size ( &rows, &cols, array2d_lower );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Test if the input array is square
   if ( rows != cols ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data, array2d_lower );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = i; j < cols; j++)
      {
         data[i][j] = data[j][i];
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_double_symmetrise_upper ( Rox_Array2D_Double array2d_upper )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !array2d_upper ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_double_get_size ( &rows, &cols, array2d_upper );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Test if the input array is square
   if ( rows != cols ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data, array2d_upper );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < i; j++)
      {
         data[i][j] = data[j][i];
      }
   }

function_terminate:
   return error;
}

