//============================================================================
//
//    OPENROX   : File remap_nn_halved.c
//
//    Contents  : Implementation of remap_nn_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "remap_nn_halved.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uchar_remap_halved_nn (
   Rox_Image dest, 
   const Rox_Image source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest)     
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uchar_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols  = cols  / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_uchar_check_size(dest, hrows, hcols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hrows; i++)
   {
      for (Rox_Sint j = 0; j < hcols; j++)
      {
         dd[i][j] = ds[i*2][j*2];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_remap_halved_nn (
   Rox_Array2D_Float dest, 
   const Rox_Array2D_Float source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_float_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_float_check_size(dest, hrows, hcols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hrows; i++)
   {
      for (Rox_Sint j = 0; j < hcols; j++)
      {
         dd[i][j] = ds[i*2][j*2];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_remap_halved_nn (
   Rox_Array2D_Uint dest, 
   const Rox_Array2D_Uint source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dest)     
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)};
   
   if (!source)   
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uint_get_size(&rows, &cols, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_uint_check_size(dest, hrows, hcols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dd, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &ds, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hrows; i++)
   {
      for (Rox_Sint j = 0; j < hcols; j++)
      {
         dd[i][j] = ds[i*2][j*2];
      }
   }

function_terminate:
   return error;
}
