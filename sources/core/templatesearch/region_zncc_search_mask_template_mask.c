//============================================================================
//
//    OPENROX   : File region_zncc_search_mask_template_mask.c
//
//    Contents  : Implementation of region_zncc_search_mask_template_mask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "region_zncc_search_mask_template_mask.h"
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>


Rox_ErrorCode rox_array2d_float_region_zncc_search_mask_template_mask (
   Rox_Float * res_score, 
   Rox_Sint * res_topleft_x, 
   Rox_Sint * res_topleft_y, 
   const Rox_Array2D_Float isearch, 
   const Rox_Imask isearch_mask, 
   const Rox_Array2D_Float itemplate, 
   const Rox_Imask itemplate_mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint twidth, theight;
   Rox_Sint swidth, sheight;
   //Rox_Sint shiftwidth, shiftheight;
   //Rox_Sint shiftx, shifty;

   if (!res_score || !res_topleft_x || !res_topleft_y) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!itemplate || !itemplate_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!isearch || !isearch_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_get_size(&theight, &twidth, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_size(&sheight, &swidth, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(itemplate_mask, theight, twidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(isearch_mask, sheight, swidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (swidth < twidth || sheight < theight) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //shiftwidth = swidth - twidth;
   //shiftheight = sheight - theight;

   Rox_Uint ** dsm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dsm, isearch_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dtm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dtm, itemplate_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** ds  = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &ds, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** dt  = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dt, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_region_zncc_search_mask_template_mask (
               res_score,
               res_topleft_x,
               res_topleft_y,
               ds,
               dsm,
               sheight,
               swidth,
               dt,
               dtm,
               theight,
               twidth
   );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_region_zncc_search_mask_template_mask (
   Rox_Float * res_score, 
   Rox_Sint * res_topleft_x, 
   Rox_Sint * res_topleft_y, 
   Rox_Image isearch, 
   Rox_Imask isearch_mask, 
   Rox_Image itemplate, 
   Rox_Imask itemplate_mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint twidth, theight;
   Rox_Sint swidth, sheight;

   if (!res_score || !res_topleft_x || !res_topleft_y) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!itemplate || !itemplate_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!isearch || !isearch_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&theight, &twidth, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_size(&sheight, &swidth, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(itemplate_mask, theight, twidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(isearch_mask, sheight, swidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (swidth < twidth || sheight < theight) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** dsm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dsm, isearch_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dtm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dtm, itemplate_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar ** dt = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dt, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_uchar_region_zncc_search_mask_template_mask (
            res_score,
            res_topleft_x,
            res_topleft_y,
            ds,
            dsm,
            sheight,
            swidth,
            dt,
            dtm,
            theight,
            twidth
   );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
