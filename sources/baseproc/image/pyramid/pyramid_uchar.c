//==============================================================================
//
//    OPENROX   : File pyramid_uchar.c
//
//    Contents  : Implementation of pyramid_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "pyramid_uchar.h"
#include "pyramid_uchar_struct.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
#include <baseproc/image/remap/remap_nn_halved/remap_nn_halved.h>
#include <baseproc/image/convolve/array2d_uchar_symmetric_separable_convolve.h>
#include <baseproc/maths/kernels/gaussian2d.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pyramid_uchar_new(Rox_Pyramid_Uchar * obj, Rox_Sint width, Rox_Sint height, Rox_Uint max_levels, Rox_Uint min_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Uchar ret = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (width < 4 || height < 4 || max_levels < 1 || min_size < 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint bestsize = 0;
   error = rox_pyramid_compute_optimal_level_count(&bestsize, width, height, min_size);
   ROX_ERROR_CHECK_TERMINATE ( error );

   bestsize++; // Count also base level
   if (max_levels < bestsize) bestsize = max_levels; // Threshold

   ret = (Rox_Pyramid_Uchar) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->base_height = height;
   ret->base_width  = width;
   ret->nb_levels   = bestsize;

   // Allocate pointers
   ret->levels = (Rox_Image*) rox_memory_allocate(sizeof(Rox_Image), bestsize);
   if (!ret->levels)
   {
      rox_memory_delete(ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Allocate fast access pointers
   ret->fast_access = (Rox_Uchar***)rox_memory_allocate(sizeof(Rox_Uchar**), bestsize);
   if (!ret->fast_access)
   {
      rox_memory_delete(ret->levels);
      rox_memory_delete(ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Allocate images per level
   for (Rox_Uint i = 0; i < bestsize; i++)
   {
      error = rox_array2d_uchar_new(&ret->levels[i], height, width);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Assign fast access
      error = rox_array2d_uchar_get_data_pointer_to_pointer( &ret->fast_access[i], ret->levels[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Half the size for next level
      height /= 2;
      width /= 2;
   }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uchar_del(Rox_Pyramid_Uchar * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Uchar todel = NULL;

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
         rox_array2d_uchar_del(&todel->levels[i]);
      }

      // Delete levels container
      rox_memory_delete(todel->levels);
   }

   // Delete object
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uchar_assign(Rox_Pyramid_Uchar obj, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_check_size(source, obj->base_height, obj->base_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First level is a a copy
   error = rox_array2d_uchar_copy(obj->levels[0], source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 1; i < obj->nb_levels; i++)
   {
      // Box filtering previous level
      error = rox_remap_box_nomask_uchar_to_uchar_halved(obj->levels[i], obj->levels[i-1]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uchar_assign_gaussian(Rox_Pyramid_Uchar obj, Rox_Image source, Rox_Float sigma)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float hkernel, vkernel;

   if (!obj || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_check_size(source, obj->base_height, obj->base_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_kernelgen_gaussian2d_separable_float_new(&hkernel, &vkernel, sigma, 4.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First level is a a copy
   error = rox_array2d_uchar_copy(obj->levels[0], source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Errors are fatal and should not occur, here for debug mainly, so we don't care about freeing memory after error

   for (Rox_Uint i = 1; i < obj->nb_levels; i++)
   {
      Rox_Image buffer = NULL;

      // Create smoothing buffer
      Rox_Sint pw = 0, ph = 0;
      error = rox_array2d_uchar_get_size(&ph, &pw, obj->levels[i-1]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uchar_new(&buffer, ph, pw);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // gaussian filtering previous level
      error = rox_array2d_uchar_symmetric_seperable_convolve(buffer, obj->levels[i-1], hkernel);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Half sample
      error = rox_array2d_uchar_remap_halved_nn(obj->levels[i], buffer);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Delete buffer
      rox_array2d_uchar_del(&buffer);
   }

   rox_array2d_float_del(&hkernel);
   rox_array2d_float_del(&vkernel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uchar_get_image(Rox_Image * image, const Rox_Pyramid_Uchar pyramid, const Rox_Sint level)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pyramid || !image)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(level < 0 || level >= (Rox_Sint) pyramid->nb_levels)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *image = pyramid->levels[level];

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_uchar_get_nb_levels(Rox_Sint * nb_levels, const Rox_Pyramid_Uchar pyramid)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pyramid || !nb_levels)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *nb_levels = pyramid->nb_levels;

function_terminate:
   return error;
}
