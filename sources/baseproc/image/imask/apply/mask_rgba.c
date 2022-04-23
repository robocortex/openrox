//==============================================================================
//
//    OPENROX   : File mask_rgba.h
//
//    Contents  : API of mask_rgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mask_rgba.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_mask_alpha_roxrgba (
   Rox_Array2D_Uchar source, 
   const Rox_Array2D_Uint mask, 
   const Rox_Char value
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!source || !mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols/4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Uint * rm = dm[i];
      Rox_Uchar * rs = ds[i];
      
      for (Rox_Sint j = 0; j < cols/4; j++)
      {
         if(rm[j] == 0)
            rs[3] = value;
         
         rs += 4;
      }
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_mask_alpha_roxrgba_reverse (
   Rox_Array2D_Uchar source, 
   const Rox_Array2D_Uint mask, 
   const Rox_Char value
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!source || !mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   error = rox_array2d_uchar_get_size(&rows, &cols, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(mask, rows, cols/4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Uint  * rm = dm[i];
      Rox_Uchar * rs = ds[i];
      
      for (Rox_Sint j = 0; j < cols; j++)
      {
         if(rm[j] != 0)
            rs[3] = value;
         
         rs += 4;
      }
   }
   
function_terminate:
   return error;
}