//==============================================================================
//
//    OPENROX   : File patchplane_robustlight.c
//
//    Contents  : Implementation of patchplane_robustlight module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "patchplane_robustlight.h"

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scaleshift.h>
#include <baseproc/array/mean/mean.h>
#include <baseproc/array/band/band.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/image/gradient/basegradient.h>
#include <baseproc/array/crosscor/zncrosscor.h>
#include <baseproc/array/error/l2_error.h>
#include <baseproc/array/meanvar/meanvar.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <inout/system/errors_print.h>


Rox_ErrorCode rox_patchplane_robustlight_new(Rox_PatchPlane_RobustLight *obj, Rox_Sint height, Rox_Sint width)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PatchPlane_RobustLight ret;
   Rox_Uint bc_width, bc_height, countblocks;
   Rox_Uint last_width, last_height;
   Rox_Uint curw, curh;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_PatchPlane_RobustLight) rox_memory_allocate(sizeof(struct Rox_PatchPlane_RobustLight_Struct), 1);

   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->reference = NULL;
   ret->reference_mask = NULL;
   ret->grid = NULL;
   ret->current = NULL;
   ret->current_mask = NULL;
   ret->gx = NULL;
   ret->gy = NULL;
   ret->gradient_mask = NULL;
   ret->warped_lum = NULL;
   ret->difference = NULL;
   ret->mean = NULL;
   ret->mean_lum = NULL;
   ret->alphas = NULL;
   ret->toplefts = NULL;
   ret->subs_current = NULL;
   ret->subs_warped_lum = NULL;
   ret->subs_mean = NULL;
   ret->subs_difference = NULL;
   ret->subs_gx = NULL;
   ret->subs_gy = NULL;
   ret->subs_gradient_mask = NULL;
   ret->subs_current_mask = NULL;
   ret->pixelmeans = NULL;

   bc_width = width / 16;
   last_width = width % 16;
   if (last_width > 0) bc_width++;
   else last_width = 16;
   bc_height = height / 16;
   last_height = height % 16;
   if (last_height > 0) bc_height++;
   else last_height = 16;
   countblocks = bc_width * bc_height;


   error = rox_array2d_float_new(&ret->reference, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->reference_mask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&ret->grid, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->current, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->gx, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->gy, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->gradient_mask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->warped_lum, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->current_mask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->difference, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->mean, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->mean_lum, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&ret->alphas, countblocks + 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&ret->pixelmeans, countblocks + 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new(&ret->toplefts, countblocks + 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_float_new(&ret->subs_current, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_float_new(&ret->subs_warped_lum, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_float_new(&ret->subs_mean, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_float_new(&ret->subs_difference, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_float_new(&ret->subs_gx, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_float_new(&ret->subs_gy, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_uint_new(&ret->subs_gradient_mask, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_uint_new(&ret->subs_current_mask, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create blocks
   for ( Rox_Uint i = 0; i < bc_height; i++)
   {
      Rox_Point2D_Double_Struct pt;

      pt.v = i * 16;
      curh = 16;
      if (i == bc_height - 1) curh = last_height;

      for ( Rox_Uint j = 0; j < bc_width; j++)
      {
         Rox_Array2D_Float sub;
         Rox_Imask subui;
         Rox_Double onealpha = 1.0;

         pt.u = j * 16;
         curw = 16;
         if (j == bc_width - 1) curw = last_width;


         error = rox_array2d_float_new_subarray2d(&sub, ret->current, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_float_append(ret->subs_current, sub);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_float_new_subarray2d(&sub, ret->warped_lum, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_float_append(ret->subs_warped_lum, sub);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_float_new_subarray2d(&sub, ret->mean, (Rox_Sint) pt.v, (Rox_Sint)pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_float_append(ret->subs_mean, sub);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_float_new_subarray2d(&sub, ret->difference, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_float_append(ret->subs_difference, sub);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_float_new_subarray2d(&sub, ret->gx, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_float_append(ret->subs_gx, sub);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_float_new_subarray2d(&sub, ret->gy, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_float_append(ret->subs_gy, sub);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_uint_new_subarray2d(&subui, ret->gradient_mask, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_uint_append(ret->subs_gradient_mask, subui);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_uint_new_subarray2d(&subui, ret->current_mask, (Rox_Sint) pt.v, (Rox_Sint) pt.u, curh, curw);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_uint_append(ret->subs_current_mask, subui);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_dynvec_double_append(ret->alphas, &onealpha);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_dynvec_double_append(ret->pixelmeans, &onealpha);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_dynvec_point2d_double_append(ret->toplefts, &pt);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   ret->width = width;
   ret->height = height;

   error = rox_patchplane_robustlight_reset_luminance(ret);
   ROX_ERROR_CHECK_TERMINATE(error)

   *obj = ret;

function_terminate:
   if (error) rox_patchplane_robustlight_del(&ret);
   return error;
}

Rox_ErrorCode rox_patchplane_robustlight_del(Rox_PatchPlane_RobustLight *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PatchPlane_RobustLight todel = NULL;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_float_del(&todel->reference);
   rox_array2d_uint_del(&todel->reference_mask);
   rox_array2d_uint_del(&todel->current_mask);
   rox_meshgrid2d_float_del(&todel->grid);
   rox_array2d_float_del(&todel->current);
   rox_array2d_float_del(&todel->gx);
   rox_array2d_float_del(&todel->gy);
   rox_array2d_uint_del(&todel->gradient_mask);
   rox_array2d_float_del(&todel->warped_lum);
   rox_array2d_float_del(&todel->difference);
   rox_array2d_float_del(&todel->mean);
   rox_array2d_float_del(&todel->mean_lum);
   rox_objset_array2d_float_del(&todel->subs_current);
   rox_objset_array2d_float_del(&todel->subs_warped_lum);
   rox_objset_array2d_float_del(&todel->subs_mean);
   rox_objset_array2d_float_del(&todel->subs_difference);
   rox_objset_array2d_float_del(&todel->subs_gx);
   rox_objset_array2d_float_del(&todel->subs_gy);
   rox_objset_array2d_uint_del(&todel->subs_gradient_mask);
   rox_objset_array2d_uint_del(&todel->subs_current_mask);
   rox_dynvec_double_del(&todel->alphas);
   rox_dynvec_double_del(&todel->pixelmeans);
   rox_dynvec_point2d_double_del(&todel->toplefts);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_robustlight_set_reference(
   Rox_PatchPlane_RobustLight obj, Rox_Array2D_Float source, Rox_Imask sourcemask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source || !sourcemask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_copy(obj->reference, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_copy(obj->reference_mask, sourcemask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_robustlight_reset_luminance(Rox_PatchPlane_RobustLight obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint i = 0; i < obj->alphas->used; i++) obj->alphas->data[i] = 1.0;

   obj->beta = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_robustlight_prepare_sl3(Rox_PatchPlane_RobustLight obj, Rox_MatSL3 homography, Rox_Array2D_Float source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !homography || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp
   error = rox_warp_grid_sl3_float(obj->grid, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_bilinear_omo_float_to_float(obj->current, obj->current_mask, source, obj->grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Merge masks
   error = rox_array2d_uint_band(obj->current_mask, obj->current_mask, obj->reference_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_robustlight_prepare_finish(Rox_PatchPlane_RobustLight obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Luminance
   for ( Rox_Uint idblock = 0; idblock < obj->alphas->used; idblock++)
   {
      Rox_Double mean = 0.0;
      Rox_Double variance = 0.0;

      error = rox_array2d_float_meanvar(&mean, &variance, obj->subs_current->data[idblock], obj->subs_current_mask->data[idblock]);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_float_scaleshift(obj->subs_warped_lum->data[idblock], obj->subs_current->data[idblock], (Rox_Float) obj->alphas->data[idblock], obj->beta);
      ROX_ERROR_CHECK_TERMINATE ( error );

      obj->pixelmeans->data[idblock] = mean;
   }

   // Mean

   error = rox_array2d_float_mean(obj->mean, obj->reference, obj->current);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_mean(obj->mean_lum, obj->reference, obj->warped_lum);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Difference
   error = rox_array2d_float_substract(obj->difference, obj->reference, obj->warped_lum);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Gradient
   error = rox_array2d_float_basegradient(obj->gx, obj->gy, obj->gradient_mask, obj->mean_lum, obj->current_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_l2_error(&obj->error, obj->difference, obj->gradient_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_robustlight_compute_score(Rox_Double *score, Rox_PatchPlane_RobustLight obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!score || !obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_zncc_normalizedscore(score, obj->reference, obj->warped_lum, obj->current_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
