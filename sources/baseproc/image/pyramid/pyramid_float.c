//==============================================================================
//
//    OPENROX   : File pyramid_float.c
//
//    Contents  : Implementation of pyramid_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "pyramid_float.h"

#include <generated/array2d_float.h>

#include <system/time/timer.h>
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
#include <baseproc/image/remap/remap_nn_halved/remap_nn_halved.h>
#include <baseproc/image/convolve/array2d_float_symmetric_separable_convolve.h>
#include <baseproc/maths/kernels/gaussian2d.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_pyramid_float_new (
   Rox_Pyramid_Float * pyramid,
   Rox_Sint width,
   Rox_Sint height,
   const Rox_Sint max_levels,
   const Rox_Sint min_size
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Float ret = NULL;
   Rox_Uint bestsize = 0;

   if (!pyramid)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (width < 4 || height < 4 || max_levels < 1 || min_size < 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_pyramid_compute_optimal_level_count(&bestsize, width, height, min_size);
   ROX_ERROR_CHECK_TERMINATE ( error );

   bestsize++; //Count also base level
   if (max_levels < (Rox_Sint) bestsize) bestsize = (Rox_Uint) max_levels; // Threshold

   ret = (Rox_Pyramid_Float) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->base_height = height;
   ret->base_width = width;
   ret->nb_levels = bestsize;

   // Allocate pointers
   ret->levels = (Rox_Image_Float*) rox_memory_allocate(sizeof(Rox_Image_Float), bestsize);
   if (!ret->levels)
   { rox_memory_delete(ret);
     {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Allocate fast access pointers
   ret->fast_access = (Rox_Float***) rox_memory_allocate(sizeof(Rox_Float**), bestsize);
   if (!ret->fast_access)
   {
      rox_memory_delete(ret->levels);
      rox_memory_delete(ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Allocate images per level
   for (Rox_Uint i = 0; i < bestsize; i++)
   {
      error = rox_array2d_float_new(&ret->levels[i], height, width);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Assign fast access
      error = rox_array2d_float_get_data_pointer_to_pointer(&ret->fast_access[i], ret->levels[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Half the size for next level
      height /= 2;
      width /= 2;
   }

   *pyramid = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_float_del(Rox_Pyramid_Float * pyramid)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Float todel = NULL;

   if (!pyramid)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *pyramid;
   *pyramid = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Delete pointers
   rox_memory_delete(todel->fast_access);

   if (todel->levels)
   {
      //Delete levels
      for (Rox_Uint i = 0; i < todel->nb_levels; i++)
      {
         rox_array2d_float_del(&todel->levels[i]);
      }

      //Delete levels container
      rox_memory_delete(todel->levels);
   }

   //Delete pyramidect
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_float_assign_nofiltering ( Rox_Pyramid_Float pyramid, const Rox_Image_Float source )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pyramid || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_check_size(source, pyramid->base_height, pyramid->base_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First level is a a copy
   error = rox_array2d_float_copy(pyramid->levels[0], source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 1; i < pyramid->nb_levels; i++)
   {
      // Remap previous level with Neirest Neighbour
      error = rox_array2d_float_remap_halved_nn(pyramid->levels[i], pyramid->levels[i-1]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_float_assign ( Rox_Pyramid_Float pyramid, const Rox_Image_Float source )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pyramid || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_check_size(source, pyramid->base_height, pyramid->base_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First level is a a copy
   error = rox_array2d_float_copy ( pyramid->levels[0], source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 1; i < pyramid->nb_levels; i++)
   {
      // Box filtering previous level
      error = rox_remap_box_nomask_float_to_float_halved(pyramid->levels[i], pyramid->levels[i-1]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_float_assign_gaussian(Rox_Pyramid_Float pyramid, const Rox_Image_Float source, const Rox_Float sigma)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image_Float hkernel, vkernel;

   if (!pyramid || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_check_size(source, pyramid->base_height, pyramid->base_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_kernelgen_gaussian2d_separable_float_new(&hkernel, &vkernel, sigma, 4.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First level is a a copy
   error = rox_array2d_float_copy(pyramid->levels[0], source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Errors are fatal and should not occur, here for debug mainly, so we don't care about freeing memory after error
   for (Rox_Uint i = 1; i < pyramid->nb_levels; i++)
   {
      Rox_Image_Float buffer = NULL;
      Rox_Sint pw = 0, ph = 0;

      //Create smoothing buffer
      error = rox_array2d_float_get_size(&ph, &pw, pyramid->levels[i-1]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_new(&buffer, ph, pw);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Gaussian filtering previous level
      error = rox_array2d_float_symmetric_seperable_convolve(buffer, pyramid->levels[i-1], hkernel);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Half sample
      error = rox_array2d_float_remap_halved_nn(pyramid->levels[i], buffer);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Delete buffer
      rox_array2d_float_del(&buffer);
   }

function_terminate:
   rox_array2d_float_del(&hkernel);
   rox_array2d_float_del(&vkernel);
   return error;
}

Rox_ErrorCode rox_pyramid_float_get_image(Rox_Image_Float * image, const Rox_Pyramid_Float pyramid, const Rox_Sint level)
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

Rox_ErrorCode rox_pyramid_float_get_nb_levels(Rox_Sint * nb_levels, const Rox_Pyramid_Float pyramid)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pyramid || !nb_levels)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *nb_levels = pyramid->nb_levels;

function_terminate:
   return error;
}
