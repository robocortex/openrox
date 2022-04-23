//==============================================================================
//
//    OPENROX   : File patchplane_pyramid.c
//
//    Contents  : Implementation of patchplane_pyramid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "patchplane_pyramid.h"
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
#include <baseproc/image/remap/remap_box_mask_halved/remap_box_mask_halved.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_patchplane_pyramid_new(Rox_PatchPlane_Pyramid *obj, Rox_Sint height, Rox_Sint width)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PatchPlane_Pyramid ret = NULL;
   Rox_Uint id = 0 ;
   Rox_Double cwidth = 0.0, cheight = 0.0;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *obj = NULL;

   ret = (Rox_PatchPlane_Pyramid) rox_memory_allocate(sizeof(*ret), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->levels = NULL;

   // Compute optimal pyramid size
   error = rox_pyramid_compute_optimal_level_count ( &ret->count, width, height, 25 );
   if (error)
   { ret->count = 0; }

   // Add base level
   ret->count++;

   ret->levels = (Rox_PatchPlane *) rox_memory_allocate(sizeof(Rox_PatchPlane), ret->count);
   if (!ret->levels)
   {
      rox_patchplane_pyramid_del(&ret);

      error = ROX_ERROR_NULL_POINTER; 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   for (id = 0; id < ret->count; id++)
   {
      ret->levels[id] = NULL;
   }

   cwidth = width;
   cheight = height;
   for (id = 0; id < ret->count; id++)
   {
      error = rox_patchplane_new(&ret->levels[id], (int) cheight, (int) cwidth);
      if (error)
      {
         rox_patchplane_pyramid_del(&ret);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      cheight /= 2.0;
      cwidth /= 2.0;
   }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_pyramid_del(Rox_PatchPlane_Pyramid *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PatchPlane_Pyramid todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (todel->levels)
   {
      for (Rox_Uint id = 0; id < todel->count; id++) rox_patchplane_del(&todel->levels[id]);
      rox_memory_delete(todel->levels);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_pyramid_apply(Rox_PatchPlane_Pyramid obj, Rox_Array2D_Float source, Rox_Imask mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_copy(obj->levels[0]->reference, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!mask)
   {
      error = rox_array2d_uint_fillval(obj->levels[0]->reference_mask, ~0); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_array2d_uint_copy(obj->levels[0]->reference_mask, mask); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   for (Rox_Uint i = 1; i < obj->count; i++)
   {
      error = rox_remap_box_nomask_float_to_float_halved(obj->levels[i]->reference, obj->levels[i - 1]->reference); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_uint_remap_halved_mask(obj->levels[i]->reference_mask, obj->levels[i - 1]->reference_mask); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_pyramid_reset_luminance(Rox_PatchPlane_Pyramid obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < obj->count; i++)
   {
      error = rox_patchplane_reset_luminance(obj->levels[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
