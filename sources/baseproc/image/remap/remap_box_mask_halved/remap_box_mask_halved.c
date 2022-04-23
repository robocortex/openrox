//============================================================================
//
//    OPENROX   : File remap_box_mask_halved.c
//
//    Contents  : Implementation of remap_box_mask_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "remap_box_mask_halved.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_remap_halved_mask (
   Rox_Imask imask_out, 
   const Rox_Imask imask_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!imask_out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!imask_inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, imask_inp); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint hcols = cols / 2;
   Rox_Sint hrows = rows / 2;

   error = rox_array2d_uint_check_size(imask_out, hrows, hcols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dd = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dd, imask_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** ds = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &ds, imask_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hrows; i++)
   {
      for (Rox_Sint j = 0; j < hcols; j++)
      {
         dd[i][j] = 0;

         if (!ds[i * 2][j * 2]) continue;
         if (!ds[i * 2][j * 2 + 1]) continue;
         if (!ds[i * 2 + 1][j * 2]) continue;
         if (!ds[i * 2 + 1][j * 2 + 1]) continue;

         dd[i][j] = ~0;
      }
   }
   
function_terminate:
   return error;
}
