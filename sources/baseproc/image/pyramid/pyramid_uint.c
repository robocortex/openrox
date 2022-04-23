//==============================================================================
//
//    OPENROX   : File pyramid_uint.c
//
//    Contents  : Implementation of pyramid_uint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "pyramid_uint.h"

#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
#include <baseproc/image/remap/remap_nn_halved/remap_nn_halved.h>
#include <baseproc/maths/kernels/gaussian2d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pyramid_uint_new(Rox_Pyramid_Uint * obj, Rox_Sint width, Rox_Sint height, Rox_Uint max_levels, Rox_Uint min_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Uint ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (width < 4 || height < 4 || max_levels < 1 || min_size < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); } 
   
   Rox_Uint bestsize = 0;
   error = rox_pyramid_compute_optimal_level_count(&bestsize, width, height, min_size); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   bestsize++; // Count also base level
   if (max_levels < bestsize) bestsize = max_levels; // Threshold

   ret = (Rox_Pyramid_Uint)rox_memory_allocate(sizeof(struct Rox_Pyramid_Uint_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->base_height = height;
   ret->base_width = width;
   ret->nb_levels = bestsize;

   // Allocate pointers
   ret->levels = (Rox_Array2D_Uint*)rox_memory_allocate(sizeof(Rox_Array2D_Uint), bestsize);
   if (!ret->levels)
   {
      rox_memory_delete(ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Allocate fast access pointers
   ret->fast_access = (Rox_Uint***)rox_memory_allocate(sizeof(Rox_Uint**), bestsize);
   if (!ret->fast_access)
   {
      rox_memory_delete(ret->levels);
      rox_memory_delete(ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Allocate images per level
   for (Rox_Uint i = 0; i < bestsize; i++)
   {
      error = rox_array2d_uint_new(&ret->levels[i], height, width);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Assign fast access
      error = rox_array2d_uint_get_data_pointer_to_pointer(&ret->fast_access[i], ret->levels[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Half the size for next level
      height /= 2;
      width /= 2;
   }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uint_del(Rox_Pyramid_Uint * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Uint todel = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Delete pointers
   rox_memory_delete(todel->fast_access);

   if (todel->levels)
   {
      // Delete levels
      for (Rox_Uint i = 0; i < todel->nb_levels; i++)
      {
         rox_array2d_uint_del(&todel->levels[i]);
      }

      // Delete levels container
      rox_memory_delete(todel->levels);
   }

   // Delete object
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uint_assign_nofiltering(Rox_Pyramid_Uint obj, Rox_Array2D_Uint source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_check_size(source, obj->base_height, obj->base_width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First level is a a copy
   error = rox_array2d_uint_copy(obj->levels[0], source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 1; i < obj->nb_levels; i++)
   {
      // Box filtering previous level
      error = rox_array2d_uint_remap_halved_nn(obj->levels[i], obj->levels[i-1]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
