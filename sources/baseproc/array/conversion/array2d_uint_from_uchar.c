//============================================================================
//
//    OPENROX   : File array2d_uint_from_uchar.c
//
//    Contents  : Implementation of array2d_uint_from_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "array2d_uint_from_uchar.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_from_uchar_mask(Rox_Array2D_Uint dest, Rox_Array2D_Uchar source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest || ! source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_match_size((Rox_Array2D)dest, (Rox_Array2D)source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint   **dd = NULL; 
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar  **ds = NULL; 
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   
   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   for ( Rox_Sint i = 0; i < rows; i++ )
   {
       for ( Rox_Sint j = 0; j < cols; j++ )
       {
          Rox_Uchar val = ds[i][j];
          dd[i][j] = val << 24 | val << 16 | val << 8 | val;
       }
   }

function_terminate:
   return error;
}
