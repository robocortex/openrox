//============================================================================
//
//    OPENROX   : File remap_box_halved.c
//
//    Contents  : Implementation of remap_box_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "remap_box_halved.h"
#include "ansi_remap_box_halved.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_box_nomask_uchar_to_uchar_halved (
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

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_uchar_check_size ( dest, hrows, hcols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_remap_box_nomask_uchar_to_uchar_halved ( dd, ds, hrows, hcols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_remap_box_nomask_float_to_float_halved (
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
   error = rox_array2d_float_get_size ( &rows, &cols, source ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_float_check_size ( dest, hrows, hcols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dd, dest );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &ds, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_remap_box_nomask_float_to_float_halved ( dd, ds, hrows, hcols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
