//==============================================================================
//
//    OPENROX   : File ident_texture_sl3.c
//
//    Contents  : Implementation of ident_texture_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_texture_sl3.h"

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>

#include <core/identification/templateident_sl3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ident_texture_sl3_new(Rox_Ident_Texture_SL3 *ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Texture_SL3 ret = NULL;
   
   if(!ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ident = NULL;

   ret = (Rox_Ident_Texture_SL3) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_sl3_new(&ret->identifier);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->dbl_size = 0;

   *ident = ret;

function_terminate:
   // Delete only if an error occurs
   if(error) rox_ident_texture_sl3_del(&ret);

   return error;
}

Rox_ErrorCode rox_ident_texture_sl3_del(Rox_Ident_Texture_SL3 * ident)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Texture_SL3 todel = NULL;

   if(!ident) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident;
   *ident = NULL;

   if(!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_template_ident_sl3_del(&todel->identifier);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_texture_sl3_set_model (
   Rox_Ident_Texture_SL3 ident, 
   Rox_Image model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_model = NULL;

   if(!ident || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&norm_model, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_from_uchar_normalize(norm_model, model); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_template_ident_sl3_reset(ident->identifier);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_template_ident_sl3_add_model_affine(ident->identifier, norm_model, ident->dbl_size); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_template_ident_sl3_compile(ident->identifier); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_model);
   return error;
}

Rox_ErrorCode rox_ident_texture_sl3_make (
   Rox_Sint * is_identified, 
   Rox_MatSL3 homography, 
   Rox_Ident_Texture_SL3 ident, 
   Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_cur = NULL;
   Rox_Uint id = 200;

   if(!homography || !ident || !image || !is_identified) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = 0;

   Rox_Sint rows = 0, cols = 0;
   error = rox_image_get_size(&rows, &cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&norm_cur, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_from_uchar_normalize(norm_cur, image); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_template_ident_sl3_make(ident->identifier, norm_cur, ident->dbl_size); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_template_ident_sl3_get_next_best_homography(homography, &id, ident->identifier); 
   if(!error)
   {
     *is_identified = 1;
   }
   if(error == ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE) {error = ROX_ERROR_NONE;}
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_cur);
   return error;
}

Rox_ErrorCode rox_ident_texture_sl3_enable_double_size(Rox_Ident_Texture_SL3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->dbl_size = 1;
 
function_terminate:
   return error;
}
