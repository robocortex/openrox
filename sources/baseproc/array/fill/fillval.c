//==============================================================================
//
//    OPENROX   : File fillval.c
//
//    Contents  : Implementation of fillval module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fillval.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_fillval(Rox_Array2D_Double dest, const Rox_Double value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = value;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_fillval(Rox_Array2D_Float dest, const Rox_Float value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, dest ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &data, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = value;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_sint_fillval(Rox_Array2D_Sint dest, const Rox_Sint value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_sint_get_size ( &rows, &cols, dest ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint ** data = NULL;
   error = rox_array2d_sint_get_data_pointer_to_pointer ( &data, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = value;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_fillval(Rox_Array2D_Uint dest, const Rox_Uint value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size ( &rows, &cols, dest ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

    for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = value;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_fillval(Rox_Array2D_Uchar dest, const Rox_Uchar value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

    for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = value;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_sshort_fillval(Rox_Array2D_Sshort dest, const Rox_Sshort value)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_sshort_get_size ( &rows, &cols, dest ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sshort ** data = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer ( &data, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

    for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         data[i][j] = value;
      }
   }

function_terminate:
   return error;
}
