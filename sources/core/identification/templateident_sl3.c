//==============================================================================
//
//    OPENROX   : File templateident_sl3.c
//
//    Contents  : Implementation of templateident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "templateident_sl3.h"
#include "templateident_sl3_struct.h"

#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/transforms/matsl3/sl3virtualview.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_float_to_float/remap_bilinear_float_to_float.h>
#include <baseproc/array/fill/fillval.h>
#include <core/indirect/homography/ransacsl3.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_template_ident_sl3_new(Rox_Template_Ident_SL3 *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident_SL3 ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   ret = (Rox_Template_Ident_SL3) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   ret->matcher = NULL;
   ret->current_points_ransac = NULL;
   ret->reference_points_ransac = NULL;

   error = rox_multi_ident_new(&ret->matcher); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->current_points_ransac, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->reference_points_ransac, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   // Delete only if an error occurs
   if (error) rox_template_ident_sl3_del(&ret);
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_del(Rox_Template_Ident_SL3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident_SL3 todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Delete point lists
   rox_multi_ident_del(&todel->matcher);
   rox_dynvec_point2d_float_del(&todel->current_points_ransac);
   rox_dynvec_point2d_float_del(&todel->reference_points_ransac);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_reset(Rox_Template_Ident_SL3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Delete point lists
   error = rox_multi_ident_reset(obj->matcher);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_add_model(Rox_Template_Ident_SL3 obj, Rox_Array2D_Float model_image, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj || !model_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_multi_ident_add_template(obj->matcher, model_image, NULL, 0, dbl_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_add_model_affine(Rox_Template_Ident_SL3 obj, Rox_Array2D_Float model_image, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj || !model_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_multi_ident_add_template(obj->matcher, model_image, NULL, 1, dbl_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_compile(Rox_Template_Ident_SL3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_multi_ident_compile(obj->matcher);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_make(Rox_Template_Ident_SL3 obj, Rox_Array2D_Float current_image, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !current_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_multi_ident_make(obj->matcher, current_image, dbl_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_sl3_get_next_best_homography(Rox_Array2D_Double homography, Rox_Uint *id, Rox_Template_Ident_SL3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !homography) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_multi_ident_find_next_best(id, obj->matcher); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Template_Ident tmp = obj->matcher->idents->data[*id];

   // Ransac pose estimation
   error = rox_ransac_homography(homography, obj->current_points_ransac, obj->reference_points_ransac, tmp->current_points_matched, tmp->reference_points_matched); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
