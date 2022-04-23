//==============================================================================
//
//    OPENROX   : File array2d_uchar_from_float.c
//
//    Contents  : Implementation of array2d_uchar_from_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_uchar_from_float.h"
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uchar_from_float ( Rox_Array2D_Uchar output, Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_match_size ( (Rox_Array2D) output, (Rox_Array2D) input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** inp = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out = NULL; 
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( !inp || !out ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, output );
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (Rox_Uchar) ((Rox_Sint) inp[i][j]);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_from_float_normalize ( Rox_Array2D_Uchar output, Rox_Array2D_Float input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_match_size((Rox_Array2D)output, (Rox_Array2D)input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** inp = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &inp, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** out = NULL; 
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &out, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!inp || !out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         out[i][j] = (Rox_Uchar) (255.0f * inp[i][j]);
      }
   }

function_terminate:
   return error;
}
