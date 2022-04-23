//==============================================================================
//
//    OPENROX   : File multiident_sl3.c
//
//    Contents  : Implementation of multiident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "multiident_sl3.h"

#include <system/memory/memory.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <core/identification/templateident_sl3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_multi_ident_sl3_new ( Rox_Multi_Ident_SL3 * multi_ident_sl3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Multi_Ident_SL3 ret = NULL;
   
   if (!multi_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *multi_ident_sl3 = 0;

   ret = (Rox_Multi_Ident_SL3) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_sl3_new(&ret->identifier);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *multi_ident_sl3 = ret;

function_terminate:
   // Delete only if an error occurs
   if(error) rox_multi_ident_sl3_del(&ret);
   return error;
}

Rox_ErrorCode rox_multi_ident_sl3_del(Rox_Multi_Ident_SL3 * multi_ident_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Multi_Ident_SL3 todel = NULL;

   if (!multi_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *multi_ident_sl3;
   *multi_ident_sl3 = 0;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_template_ident_sl3_del(&todel->identifier);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_sl3_add_model(Rox_Multi_Ident_SL3 multi_ident_sl3, Rox_Image model)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_model = NULL;

   if(!multi_ident_sl3 || !model) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_new_from_uchar_normalize(&norm_model, model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_template_ident_sl3_add_model_affine(multi_ident_sl3->identifier, norm_model, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_model);
   return error;
}

Rox_ErrorCode rox_multi_ident_sl3_compile ( Rox_Multi_Ident_SL3 multi_ident_sl3 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !multi_ident_sl3 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_sl3_compile ( multi_ident_sl3->identifier );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_multi_ident_sl3_make (Rox_Multi_Ident_SL3 multi_ident_sl3, Rox_Image current)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float norm_cur = NULL;

   if(!multi_ident_sl3 || !current) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_float_new_from_uchar_normalize(&norm_cur, current);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_template_ident_sl3_make(multi_ident_sl3->identifier, norm_cur, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_float_del(&norm_cur);
   return error;
}

Rox_ErrorCode rox_multi_ident_sl3_get_homography(Rox_MatSL3 homography, Rox_Uint *id, Rox_Multi_Ident_SL3 multi_ident_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography || !id || !multi_ident_sl3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_template_ident_sl3_get_next_best_homography(homography, id, multi_ident_sl3->identifier);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
